#!/usr/bin/env python3

import sys
import math
import rospy
import tf2_ros
import moveit_commander

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState

current_joint_states = None

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

def printJointAngles():
    rads = current_joint_states.position
    angles = [(float(math.degrees(r))) for r in rads]
    names = current_joint_states.name
    joint0 = names.index('iw_ankle_foot_bottom')
    joint1 = names.index('iw_beam_ankle_bottom')
    joint2 = names.index('iw_mid_joint')
    joint3 = names.index('iw_beam_ankle_top')
    joint4 = names.index('iw_ankle_foot_top')
    
    print(f"Joint 0: {angles[joint0]}")
    print(f"Joint 1: {angles[joint1]}")
    print(f"Joint 2: {angles[joint2]}")
    print(f"Joint 3: {angles[joint3]}")
    print(f"Joint 4: {angles[joint4]}")

if __name__ == "__main__":
    prefix = ""

    if rospy.get_namespace() == "/":
        prefix = "/inchworm_0/"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_ee")

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    js_sub = rospy.Subscriber(f"{prefix}joint_states", JointState, jointStateCB, queue_size=1)

    goal_pub = rospy.Publisher(f"{prefix}next_goal", PoseStamped, queue_size=1)
    traj_pub = rospy.Publisher(f"{prefix}position_trajectory_controller/command", JointTrajectory, queue_size=1)

    robot = moveit_commander.RobotCommander()

    group_name = "ltr"
    group = moveit_commander.MoveGroupCommander(group_name)

    while current_joint_states is None:
        rospy.sleep(1)

    trans = getTransform("iw_foot_bottom", "iw_foot_top", buffer, listener).transform
    (r, p, y) = transToRPY(trans)

    print(f"Reference frame: {group.get_planning_frame()}")
    print(f"End effector: {group.get_end_effector_link()}")

    print(f"Current orientation (deg):\n\tRoll: {r:.2f}\n\tPitch: {p:.2f}\n\tYaw: {y:.2f}")

    print("Following are relative to reference frame.")

    print("Position in meters:")
    x = float(input("X: "))
    y = float(input("Y: "))
    z = float(input("Z: "))

    print("Rotation in degrees:")
    roll = float(input("Roll: "))
    pitch = float(input("Pitch: "))
    yaw = float(input("Yaw: "))

    quat = rpyToQuat(roll, pitch, yaw)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "world"
    goal_pose.header.stamp = rospy.Time.now()

    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z

    goal_pose.pose.orientation.x = quat[0]
    goal_pose.pose.orientation.y = quat[1]
    goal_pose.pose.orientation.z = quat[2]
    goal_pose.pose.orientation.w = quat[3]

    goal_pub.publish(goal_pose)

    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.01)
    group.set_pose_target(goal_pose)

    group.go(wait=True)

    printJointAngles()

    # traj_pub.publish(plan[1].joint_trajectory)