#!/usr/bin/env python3

import rospy, message_filters, math

from matplotlib import pyplot as plt
from sensor_msgs.msg import JointState

timestamps = []
errors = [[] for _ in range(5)]

enabled = False

def stateCB(desired_state, actual_state):
  global timestamps, errors, enabled

  if not enabled: return

  joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

  timestamps.append(rospy.Time.now().to_time())

  for i,name in enumerate(joint_names):
      errors[i].append(math.degrees(desired_state.position[desired_state.name.index(name)] - actual_state.position[actual_state.name.index(name)]))

def main():
  global enabled

  rospy.init_node("error_plotter")

  desired_sub = message_filters.Subscriber("/inchworm/joint_goal", JointState)
  actual_sub  = message_filters.Subscriber("/inchworm/joint_states", JointState)

  time_sync = message_filters.ApproximateTimeSynchronizer([desired_sub, actual_sub], 3, 0.0075)
  time_sync.registerCallback(stateCB)

  input("press enter to start collecting data")
  enabled = True
  input("Press enter to stop collecting data")
  enabled = False

  plt.title("Joint errors for all joints")

  for i in range(5):
    plt.plot(timestamps, errors[i], label=f"Joint {i}")

  plt.legend()
  plt.xlabel("Time (s)")
  plt.ylabel("Joint error (deg)")

  plt.show()

if __name__ == "__main__":
  main()