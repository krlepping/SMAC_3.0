#!/usr/bin/env python3

import sys
import math
import rospy
import tf2_ros
import moveit_commander

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

from inchworm_hw_interface.msg import MagnetState

current_joint_states = None

ik_service_proxy = None

def jointStateCB(msg):
    global current_joint_states

    current_joint_states = msg

def getTransform(frame_from, frame_to, buffer, listener):
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

def transToRPY(trans):
    quat = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]

    (r, p, y) = [math.degrees(n) for n in euler_from_quaternion(quat)]

    return r,p,y

def rpyToQuat(roll, pitch, yaw):
    # Assume incoming is in degrees
    roll, pitch, yaw = [math.radians(n) for n in (roll, pitch, yaw)]

    quat = quaternion_from_euler(roll, pitch, yaw)

    return quat

def goToPose(pose, wait=True, tries=5):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "world"
    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.pose = pose

    goal_pub.publish(goal_pose)

    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
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
        sys.exit()
    
    group.execute(plan[1], wait=wait)

def computeIK(pose, group="ltr", guess=None, timeout=5.0):
    ik_req = GetPositionIKRequest()

    ik_req.ik_request.group_name = group

    guess_angles = guess
    if guess_angles is None:
        guess_angles = current_joint_states.position

    ik_req.ik_request.robot_state.joint_state.name = current_joint_states.name
    ik_req.ik_request.robot_state.joint_state.position = guess_angles

    ik_req.pose_stamped.header.frame_id = "world"
    ik_req.pose_stamped.header.stamp = rospy.Time.now()
    ik_req.pose_stamped.pose = pose

    ik_req.timeout = rospy.Duration(timeout)

    res = ik_service_proxy(ik_req)

    return (res.solution, res.error_code)

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ee")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    js_sub = rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB, queue_size=1)

    goal_pub = rospy.Publisher("/inchworm/next_goal", PoseStamped, queue_size=1)
    traj_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)
    mag_state_pub = rospy.Publisher("/inchworm/magnet_states", MagnetState, queue_size=1)

    ik_service_proxy = rospy.ServiceProxy("/compute_ik", GetPositionIK)

    robot = moveit_commander.RobotCommander()

    group_name = "ltr"
    group = moveit_commander.MoveGroupCommander(group_name)

    while current_joint_states is None:
        rospy.sleep(1)

    # Get the transform from the end effector to its magnet TF
    ee_to_magnet = getTransform("iw_foot_top", "iw_foot_top/male_0", buffer, listener).transform

    print("Transform from foot to magnet:")
    print(ee_to_magnet)

    shingle_pose = getTransform("world", "shingle_10/female_0", buffer, listener).transform

    ALIGNMENT_OFFSET = 0.03

    # Alignment position is just above the shingle
    alignment_pose = Pose()
    alignment_pose.position = shingle_pose.translation
    alignment_pose.orientation = shingle_pose.rotation
    
    # Offset from magnet + 3 cm
    alignment_pose.position.z -= ee_to_magnet.translation.z - ALIGNMENT_OFFSET

    rospy.logwarn("Going to approach pose")
    goToPose(alignment_pose)

    input("Waiting for input")

    rospy.logwarn("Disabling magnet")
    top_disabled = MagnetState(magnet1=True, magnet2=False)
    mag_state_pub.publish(top_disabled)

    # Drop the EE back down
    pickup_pose = alignment_pose
    pickup_pose.position.z -= ALIGNMENT_OFFSET
    
    rospy.logwarn("Approaching shingle")
    goToPose(pickup_pose)

    rospy.sleep(2)

    # Enable magnets
    rospy.logwarn("Enabling magnets")
    both_enabled = MagnetState(magnet1=True, magnet2=True)
    mag_state_pub.publish(both_enabled)

    input("Waiting for input")

    rospy.logwarn("Disabling original foot")
    bottom_disabled = MagnetState(magnet1=False, magnet2=True)
    mag_state_pub.publish(bottom_disabled)

    # away_pose = pickup_pose
    # away_pose.position.z += ALIGNMENT_OFFSET * 2

    # rospy.logwarn("Moving away")
    # goToPose(away_pose)

    # rospy.sleep(2)

    # rospy.logwarn("Disabling magnet")
    # mag_state_pub.publish(top_disabled)

    # away_pose.position.z += 0.02
    # goToPose(away_pose)