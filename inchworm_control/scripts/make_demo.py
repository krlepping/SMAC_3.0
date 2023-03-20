#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from traj_planner import TrajectoryPlanner

def main():
  rospy.init_node("make_demo")

  planner = TrajectoryPlanner()

  rospy.sleep(1.0)

  

if __name__ == "__main__":
  main()