#!/usr/bin/env python3

import rospy, math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

joint_msg = None

def jointstateCB(msg):
  global joint_msg

  joint_msg = msg

def main():
  rospy.init_node("desired_pose_startup")

  rospy.Subscriber("/inchworm/joint_states", JointState, jointstateCB)
  # heartbeat_pub = rospy.Publisher("/inchworm/heartbeat_req", JointState, latch=True)

  traj_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)

  # msg = JointState(data=5)
  # heartbeat_pub.publish(msg)

  print("Waiting for joint_state msg")

  while joint_msg is None and not rospy.is_shutdown():
    rospy.sleep(0.25)

  print("Heartbeat received")

  traj_msg = JointTrajectory()

  pt = JointTrajectoryPoint()

  angles = [0, 19.655, 140.689, 19.655, 0]

  pt.positions = [math.radians(n) for n in angles]
  pt.time_from_start = rospy.Duration(2.0)

  traj_msg.points.append(pt)
  traj_msg.joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]
  traj_msg.header.stamp = rospy.Time.now()

  traj_pub.publish(traj_msg)

  print("Sent trajectory")

if __name__ == "__main__":
  main()