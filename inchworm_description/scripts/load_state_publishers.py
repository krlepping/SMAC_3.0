#!/usr/bin/env python3

import rospy, roslaunch


def main(count):
  #rospy.init_node("load_state_publishers")

  #count = rospy.get_param("~robot_count", 1)

  launch = roslaunch.scriptapi.ROSLaunch()
  
  launch.start()

  for i in range(count):
    node = roslaunch.core.Node("robot_state_publisher", "robot_state_publisher", name=f"state_publisher_{i}", namespace=f"/inchworm_{i}", launch_prefix="bash -c 'sleep 3; $0 $@' ")
    launch.launch(node)

  # Handle Ctrl+C nicely
  try:
    launch.spin()
  finally:
    launch.shutdown()

if __name__ == "__main__":
  main(1)