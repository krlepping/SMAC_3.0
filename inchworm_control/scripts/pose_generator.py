#!/usr/bin/env python3

from pyrfc3339 import generate
import rospy, math, tf2_ros, moveit_commander, rospkg

from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from assembly_msgs.msg import MateList
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from inchworm_hw_interface.msg import MagnetState

from traj_planner import TrajectoryPlanner

trajectory_pub = None

current_joint_states = None
on_shingle = None

ik_service_proxy = None

group = None

def jointStateCB(msg):
  global current_joint_states

  current_joint_states = msg

def mateCB(msg):
  global on_shingle

  on_shingle = int(msg.female[0].split("_")[-1])

def computeIK(pose, group="ltr", guess=None, timeout=5.0):
  ik_req = GetPositionIKRequest()

  ik_req.ik_request.group_name = group

  guess_angles = guess
  if guess_angles is None:
    guess_angles = current_joint_states.position

  ik_req.ik_request.robot_state.joint_state.name = current_joint_states.name
  ik_req.ik_request.robot_state.joint_state.position = guess_angles

  ik_req.ik_request.pose_stamped.header.frame_id = "world"
  ik_req.ik_request.pose_stamped.header.stamp = rospy.Time.now()
  ik_req.ik_request.pose_stamped.pose = pose

  ik_req.ik_request.avoid_collisions = False

  ik_req.ik_request.timeout = rospy.Duration(timeout)

  res = ik_service_proxy(ik_req)

  return (res.solution, res.error_code)

def getPlanAngles(pose, tries=5, pos_tolerance=0.01, orient_tolerance=0.01):
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = "world"
  goal_pose.header.stamp = rospy.Time.now()

  goal_pose.pose = pose

  group.set_goal_position_tolerance(pos_tolerance)
  group.set_goal_orientation_tolerance(orient_tolerance)
  group.set_pose_target(goal_pose)

  foundPlan = False
  attempts = 0
  while not foundPlan or attempts > tries:
      plan = group.plan()
      if plan[0]:
          foundPlan = True
      else:
          attempts += 1

  if attempts > tries:
      rospy.logerr("Failed to find plan, quitting.")
      rospy.logwarn(pose)

      return None
  
  return plan[1]

def getTransform(frame_from, frame_to, buffer):
  have_transform = False
  trans = None

  while not have_transform:
    try:
      trans = buffer.lookup_transform(frame_from, frame_to, rospy.Time(0))
      have_transform = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as err:
      print(f"Lookup from {frame_from} to {frame_to} failed with error:")
      print(err)

      rospy.sleep(1.0)
      continue

  return trans

def idx_to_coord(index, width):
  return (index % width, math.floor(index / width))

def coord_to_idx(coord, width):
  return width*coord[1] + coord[0]

def runQuinticJointTraj(angles, duration):
  last_states = current_joint_states

  joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
  cur_angles = []

  # Reorder the joint names to be the order specified by joint_names
  for name in joint_names:
    cur_angles.append(last_states.position[last_states.name.index(name)])

  # The desired angles from the payload
  new_angles = [float(q) for q in angles]

  duration = float(duration)
  NUM_PTS = 50

  # Calculate the quintic trajectory for each joint. Impose 0 velocity and 0 acceleration at limits.
  traj_triplets = []
  for (p0, pf) in zip(cur_angles, new_angles):
    traj = TrajectoryPlanner.quintic_interp(duration, p0, pf, 0, 0, 0, 0, NUM_PTS)
    traj_triplets.append(traj)

  traj_pts = []

  for i in range(NUM_PTS):
    pt = JointTrajectoryPoint()
    pt.positions     = [traj_triplets[joint][0][i] for joint in range(5)]
    pt.velocities    = [traj_triplets[joint][1][i] for joint in range(5)]
    pt.accelerations = [traj_triplets[joint][2][i] for joint in range(5)]

    pt.time_from_start = rospy.Duration(i * (duration / NUM_PTS))

    traj_pts.append(pt)

  trajectory = JointTrajectory()
  trajectory.joint_names = joint_names
  trajectory.points = traj_pts

  trajectory.header.stamp = rospy.Time.now()
  trajectory.header.frame_id = "world"

  trajectory_pub.publish(trajectory)

  rospy.sleep(duration)

def generateConstantsClass(consts, filename="joint_consts.py", pkg="inchworm_control"):
  rospack = rospkg.RosPack()

  path = rospack.get_path(pkg)

  with open(path + "/scripts/" + filename, "w+") as f:
    f.write("class JointConstants():\n")

    for pose_name,angles in consts.items():
      f.write(f"\t{pose_name} = [{','.join([str(q) for q in angles])}]\n")

def reorderJoints(joint_names, angles):
  out = []

  # Reorder the joint names to be the order specified by joint_names
  for name in joint_names:
    out.append(angles[joint_names.index(name)])

  return out

def main():
  global ik_service_proxy, group, trajectory_pub

  rospy.init_node("pose_generator")

  rospy.Subscriber("/active_mates", MateList, mateCB)
  rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB)

  trajectory_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)
  magnet_pub = rospy.Publisher("/inchworm/magnet_states", MagnetState, queue_size=1)

  pose_pub = rospy.Publisher("/desired_pose", PoseStamped, queue_size=1)

  ik_service_proxy = rospy.ServiceProxy("/compute_ik", GetPositionIK)

  buffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(buffer)

  group_name = "ltr"
  group = moveit_commander.MoveGroupCommander(group_name)

  while on_shingle is None or current_joint_states is None:
    rospy.sleep(1)

  # Define set of transforms we want relative to shingle magnet frames
    # coincident, aligned at an offset, ???

  EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
  ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

  # TODO: Put this on the parameter server
  WIDTH = 3

  # Determine the shingle coords we are currently on based on args or parameters
  coord = idx_to_coord(on_shingle, WIDTH)

  pair_offsets = []

  if coord[1] % 2 == 0:
    pair_offsets = ODD_ROW_N_LOOKUP
  else:
    pair_offsets = EVEN_ROW_N_LOOKUP

  neighbor_idx = []

  for pair in pair_offsets:
    neighbor_idx.append(coord_to_idx((coord[0] + pair[0], coord[1] + pair[1]), WIDTH))

  ee_to_magnet = getTransform("iw_foot_top_0/male_0", "iw_foot_top", buffer)

  top_disabled = MagnetState(magnet1=True, magnet2=False)
  magnet_pub.publish(top_disabled)

  pose_names = ["right", "bottom_right", "bottom_left", "left", "upper_left", "upper_right"]

  angle_dict = {}

  # For each of the six neighbors
  for i,neighbor in enumerate(neighbor_idx):
    shingle_name = f"shingle_{neighbor}/female_0"

    # Get the pose of the shingle magnet link relative to world
    shingle_tf = getTransform("world", shingle_name, buffer)
    ee_tf = shingle_tf.transform
    ee_tf.translation.z += ee_to_magnet.transform.translation.z + 0.0137

    pose = Pose()

    pose.position = ee_tf.translation
    pose.orientation = ee_tf.rotation

    with_stamp = PoseStamped(pose=pose)
    with_stamp.header.frame_id = "world"
    with_stamp.header.stamp = rospy.Time.now()

    pose_pub.publish(with_stamp)

    res = getPlanAngles(pose)

    angles = res.joint_trajectory.points[-1].positions
    names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

    angles = reorderJoints(names, angles)

    angle_dict[pose_names[i]] = angles

    runQuinticJointTraj(angles, 5.0)

    print(f"Added {pose_names[i]}.")

    # For each in transforms
      # Run compute IK on shingle magnet pose + transform
        # (Repeat on failure for N times)
      # Save these joint angles and the mirror (for other EE)

  generateConstantsClass(angle_dict)

if __name__ == "__main__":
  main()