#!/usr/bin/env python3

from math import pi
import yaml
import time
#import rospy

from traj_planner import TrajectoryPlanner

def main():
  #rospy.init_node("running_yaml")

  # Just use 0 (idk why just do it)
  idx = int(input("Robot index: "))

  planner = TrajectoryPlanner(idx=idx)

  #rospy.sleep(1.0)
  time.sleep(10)

  file_name = str(input("Name of yaml file: "))

  with open(file_name, 'r') as file:
    move_info = yaml.safe_load_all(file)

    for move in move_info:
      print(move)
      # planner.run_quintic_traj(goals, 2.0)



if __name__ == "__main__":
  main()