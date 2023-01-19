#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState

last_state = None

def jointStateCB(msg):
  global last_state

  last_state = msg

def main():
  rospy.init_node("log_joints")

  rospy.Subscriber("/inchworm_0/joint_states", JointState, jointStateCB)

  while not rospy.is_shutdown():
    input()

    state = last_state

    joint_names = ["iw_ankle_foot_bottom_0", "iw_beam_ankle_bottom_0", "iw_mid_joint_0", "iw_beam_ankle_top_0", "iw_ankle_foot_top_0"]
    cur_angles = []

    # Reorder the joint names to be the order specified by joint_names
    for name in joint_names:
      cur_angles.append(state.position[state.name.index(name)])

    for angle in cur_angles:
      print(f"{angle:.6f}")

if __name__ == "__main__":
  main()