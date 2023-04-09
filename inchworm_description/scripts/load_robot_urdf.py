#!/usr/bin/env python3

import rospy, sys, rospkg, subprocess


def main(count):
  #rospy.init_node("control_param_loader")

  #count = count = rospy.get_param("~robot_count", 1)
  
  rospack = rospkg.RosPack()
  urdf_path = rospack.get_path("inchworm_description") + "/urdf/inchworm_description.urdf"

  for i in range(count):
    urdf = subprocess.run(["rosrun", "xacro", "xacro", urdf_path, f"idx:={i}"], capture_output=True).stdout
    urdf = urdf.decode(encoding="utf-8")

    rospy.set_param(f"/inchworm_{i}/robot_description", urdf)

  rospy.logwarn("Done loading robot URDFs.")

if __name__ == "__main__":
  main(1)