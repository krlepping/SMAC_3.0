#!/usr/bin/env python3

from math import pi
import yaml
import rospy

from traj_planner import TrajectoryPlanner

def main():
  rospy.init_node("running_yaml")

  # Just use 0 (idk why just do it)
  idx = int(input("Robot index: "))

  planner = TrajectoryPlanner(idx=idx)

  rospy.sleep(0.1)

  file_name_small = str(input("Name of yaml file: "))
  file_name_complete = "//home/krlepping/inchworm_ws/src/SMAC_3.0/inchworm_control/demos/" + file_name_small

  with open(file_name_complete, 'r') as file:
    move_info = yaml.safe_load_all(file)

    for move in move_info:
      move_name = file_name_small.replace(".yaml", "")
      move_set = move[move_name]

  for i in range(0, len(move_set)):
    cur_move = move_set[i]
    cur_move_key = list(cur_move.keys())[0]
    cur_move = cur_move[cur_move_key]
    duration = cur_move["duration"]
    goals = cur_move["payload"]
    planner.run_quintic_traj(goals, duration)
    rospy.sleep(0.1)



if __name__ == "__main__":
  main()