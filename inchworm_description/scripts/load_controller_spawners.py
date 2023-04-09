#!/usr/bin/env python3

import rospy, roslaunch


def main(count):
  #rospy.init_node("load_controller_spanwers")

  #count = rospy.get_param("~robot_count", 1)

  launch = roslaunch.scriptapi.ROSLaunch()
  
  launch.start()

  for i in range(count):
    node = roslaunch.core.Node("controller_manager", "spawner", name=f"spawner_{i}", 
                               args=f"--namespace /inchworm_{i} joint_state_controller position_trajectory_controller")
    launch.launch(node)

  # Handle Ctrl+C nicely
  try:
    launch.spin()
  finally:
    launch.stop()

if __name__ == "__main__":
  main(1)