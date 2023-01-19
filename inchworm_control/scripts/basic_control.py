#!/usr/bin/env python3

from math import pi
import rospy

from traj_planner import TrajectoryPlanner

def main():
  rospy.init_node("basic_control")

  idx = int(input("Robot index: "))

  planner = TrajectoryPlanner(idx=idx)

  rospy.sleep(1.0)

  while True:
    goals = [0] * 5

    for i in range(5):
      pos = float(input(f"Joint {i} position: "))

      # Eli can have his radians, I can have degrees
      if (abs(pos) > 2.2):
        pos = pos * pi / 180

      goals[i] = (pos)

    planner.run_quintic_traj(goals, 2.0)

if __name__ == "__main__":
  main()