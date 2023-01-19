#!/usr/bin/env python3

import numpy as np
import rospy, actionlib

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

# This is a library of trajectory generation functions.

class TrajectoryPlanner:

  def __init__(self, idx=0):
    self.idx = idx
    self.real_robot = rospy.get_param("real_robot", False)

    if self.real_robot:
      traj_cli = f"/inchworm/position_trajectory_controller/follow_joint_trajectory"
      joint_topic = f"/inchworm/joint_states"
    else:
      traj_cli = f"/inchworm_{idx}/position_trajectory_controller/follow_joint_trajectory"
      joint_topic = f"/inchworm_{idx}/joint_states"

    # self.traj_pub = rospy.Publisher(traj_topic, JointTrajectory, queue_size=1)
    self.traj_client = actionlib.SimpleActionClient(traj_cli, FollowJointTrajectoryAction)
    self.traj_client.wait_for_server()

    self.joint_sub = rospy.Subscriber(joint_topic, JointState, self.jointCB)

    self.current_joint_state = None

    self.last_desired_state = None
    rospy.sleep(0.25)

  def jointCB(self, msg):
    self.current_joint_state = msg

  @staticmethod
  def cubic_traj(t0, tf, p0, pf, v0, vf):
    M = np.array([[1, t0, t0**2, t0**3],
                  [0, 1, 2*t0, 3*t0**2],
                  [1, tf, tf**2, tf**3],
                  [0, 1, 2*tf, 3*tf**2]])

    constraints = np.array([p0, v0, pf, vf])

    return np.matmul(np.linalg.inv(M), constraints)

  @staticmethod
  def eval_cubic_pose(coeffs, t):
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3

  @staticmethod
  def eval_cubic_vel(coeffs, t):
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2

  @staticmethod
  def eval_cubic_accel(coeffs, t):
    return 2*coeffs[2] + 6*coeffs[3]*t

  @staticmethod
  def quintic_traj(t0, tf, p0, pf, v0, vf, a0, af):
    M = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                  [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                  [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                  [1, tf, tf**2, tf**3, tf**4, tf**5],
                  [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                  [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

    constraints = np.array([p0, v0, a0, pf, vf, af])

    return np.matmul(np.linalg.inv(M), constraints)

  @staticmethod
  def eval_quintic_pose(coeffs, t):
    return coeffs[0] + coeffs[1]*t + coeffs[2]*t**2 + coeffs[3]*t**3 + coeffs[4]*t**4 + coeffs[5]*t**5

  @staticmethod
  def eval_quintic_vel(coeffs, t):
    return coeffs[1] + 2*coeffs[2]*t + 3*coeffs[3]*t**2 + 4*coeffs[4]*t**3 + 5*coeffs[5]*t**4

  @staticmethod
  def eval_quintic_accel(coeffs, t):
    return 2*coeffs[2]*t + 6*coeffs[3]*t + 12*coeffs[4]*t**2 + 20*coeffs[5]*t**3

  @staticmethod
  def cubic_interp(t, p0, pf, v0, vf, num_pts):
    '''
    Generates cubic trajectory coefficients, then returns points, velocities, and accelerations interpolated along that curve. Includes endpoint
    '''

    coeffs = TrajectoryPlanner.cubic_traj(0, t, p0, pf, v0, vf)

    pts = np.linspace(0, t, num_pts)

    positions = [TrajectoryPlanner.eval_cubic_pose(coeffs, pt) for pt in pts]
    velocities = [TrajectoryPlanner.eval_cubic_vel(coeffs, pt) for pt in pts]
    accelerations = [TrajectoryPlanner.eval_cubic_accel(coeffs, pt) for pt in pts]

    return (positions, velocities, accelerations)

  @staticmethod
  def quintic_interp(t, p0, pf, v0, vf, a0, af, num_pts):
    '''
    Generates cubic trajectory coefficients, then returns points, velocities, and accelerations interpolated along that curve. Includes endpoint
    '''

    coeffs = TrajectoryPlanner.quintic_traj(0, t, p0, pf, v0, vf, a0, af)

    pts = np.linspace(0, t, num_pts)

    positions = [TrajectoryPlanner.eval_quintic_pose(coeffs, pt) for pt in pts]
    velocities = [TrajectoryPlanner.eval_quintic_vel(coeffs, pt) for pt in pts]
    accelerations = [TrajectoryPlanner.eval_quintic_accel(coeffs, pt) for pt in pts]

    return (positions, velocities, accelerations)

  def get_joint_state(self):
    last_states = self.current_joint_state

    joint_names = [f"iw_ankle_foot_bottom_{self.idx}", f"iw_beam_ankle_bottom_{self.idx}", f"iw_mid_joint_{self.idx}", f"iw_beam_ankle_top_{self.idx}", f"iw_ankle_foot_top_{self.idx}"]
    cur_angles = []

    # Reorder the joint names to be the order specified by joint_names
    for name in joint_names:
      cur_angles.append(last_states.position[last_states.name.index(name)])

    return cur_angles

  def run_quintic_traj(self, angles, duration, num_pts=50, wait=True):
    '''
    Computes and executes a quintic trajectory from the current robot position to the requested position.
    float[5] angles: The desired angles to go to, in radians.
    float duration : The amount of time the trajectory should take
    int num_pts    : The number of points to interpolate for the joint trajectory.
    bool wait      : Whether this function should block through the trajectory execution or not.
    '''

    while self.current_joint_state is None:
      rospy.sleep(0.1)

    if self.last_desired_state is None:
      last_states = self.current_joint_state
    else:
      last_states = self.last_desired_state

    if self.real_robot:
      joint_names = [f"iw_ankle_foot_bottom", f"iw_beam_ankle_bottom", f"iw_mid_joint", f"iw_beam_ankle_top", f"iw_ankle_foot_top"]
    else:
      joint_names = [f"iw_ankle_foot_bottom_{self.idx}", f"iw_beam_ankle_bottom_{self.idx}", f"iw_mid_joint_{self.idx}", f"iw_beam_ankle_top_{self.idx}", f"iw_ankle_foot_top_{self.idx}"]
    cur_angles = []

    # Reorder the joint names to be the order specified by joint_names
    for name in joint_names:
      cur_angles.append(last_states.position[last_states.name.index(name)])

    # The desired angles from the payload
    new_angles = [float(q) for q in angles]

    self.last_desired_state = JointState(name=joint_names, position=new_angles)

    # Calculate the quintic trajectory for each joint. Impose 0 velocity and 0 acceleration at limits.
    traj_triplets = []
    for (p0, pf) in zip(cur_angles, new_angles):
      traj = TrajectoryPlanner.quintic_interp(duration, p0, pf, 0, 0, 0, 0, num_pts)
      traj_triplets.append(traj)

    traj_pts = []

    # For each interpolated point, grab the position, velocity, and acceleration for each joint
    for i in range(num_pts):
      pt = JointTrajectoryPoint()
      pt.positions     = [traj_triplets[joint][0][i] for joint in range(5)]
      pt.velocities    = [traj_triplets[joint][1][i] for joint in range(5)]
      pt.accelerations = [traj_triplets[joint][2][i] for joint in range(5)]

      pt.time_from_start = rospy.Duration((i+1) * (duration / num_pts))

      traj_pts.append(pt)

    # Compose the full joint trajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    trajectory.points = traj_pts

    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = "world"

    # self.traj_pub.publish(trajectory)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    
    self.traj_client.send_goal(goal)

    if wait:
      rospy.loginfo(f"Duration: {duration}")
      rospy.sleep(duration)
      # self.traj_client.wait_for_result()
      rospy.loginfo("Done sleeping")

if __name__ == "__main__":
  # No need to import if this is run as a library
  from matplotlib import pyplot as plt

  # Simple test cases
  coeffs = TrajectoryPlanner.quintic_traj(0, 1, 10, -20, 0, 0, 0, 0)

  times = np.linspace(0, 1, 50)
  pts = TrajectoryPlanner.quintic_interp(1, 10, -20, 0, 0, 0, 0, 50)[2]

  plt.scatter(times, pts)
  plt.show()