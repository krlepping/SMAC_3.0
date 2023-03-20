#!/usr/bin/env python3

import rospy, actionlib

from inchworm_control.msg import InchwormAction, InchwormGoal

def spawnShingles(spawn, client_0):
  spawn.action_type = 3
  spawn.coord_x = 0
  spawn.coord_y = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 4
  client_0.send_goal(spawn)
  client_0.wait_for_result()


  spawn.coord_x = 0
  spawn.coord_y = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 4
  client_0.send_goal(spawn)
  client_0.wait_for_result()

  spawn.coord_x = 0
  spawn.coord_y = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 1
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 2
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 3
  client_0.send_goal(spawn)
  client_0.wait_for_result()
  spawn.coord_x = 4
  client_0.send_goal(spawn)
  client_0.wait_for_result()

def send(goal0, client_0):
  client_0.send_goal(goal0)
  client_0.wait_for_result()

def spawnOne(spawn, x, y, client_0):
  spawn.action_type = 3
  spawn.coord_x = x
  spawn.coord_y = y
  client_0.send_goal(spawn)
  client_0.wait_for_result()

def move0(goal0, x, y, ee):
  goal0.action_type = 0
  goal0.coord_x = x
  goal0.coord_y = y
  # Top end effector
  goal0.end_effector = ee

def pick0(goal0, x, y, ee):
  goal0.action_type = 1
  goal0.coord_x = x
  goal0.coord_y = y
  # Top end effector
  goal0.end_effector = ee

def place0(goal0, x, y, ee):
  goal0.action_type = 2
  goal0.coord_x = x
  goal0.coord_y = y
  # Top end effector
  goal0.end_effector = ee


def move1(goal1, x, y, ee):
  goal1.action_type = 0
  goal1.coord_x = x
  goal1.coord_y = y
  # Top end effector
  goal1.end_effector = ee

def swapEE(ee):
  if (ee == 1):
    ee = 0
  else:
    ee = 1
  return ee

def walk0(goal0, x, y, ee):
  goal0.action_type = 0
  goal0.coord_x = x
  goal0.coord_y = y
  goal0.end_effector = ee
  

def walk1(goal1, x, y, ee):
  goal1.action_type = 0
  goal1.coord_x = x
  goal1.coord_y = y
  goal1.end_effector = ee


def main():
  rospy.init_node("action_client_example")

  client_0 = actionlib.SimpleActionClient("/inchworm_action_0", InchwormAction)
  client_0.wait_for_server()

  client_1 = actionlib.SimpleActionClient("/inchworm_action_1", InchwormAction)
  client_1.wait_for_server()

  rospy.loginfo("Got servers, sending goals.")

  

  goal0 = InchwormGoal()
  goal1 = InchwormGoal()
  spawn = InchwormGoal()

  # rospy.loginfo("Spawning shingles 5x5 roof start")
  # spawnShingles(spawn, client_0)
  # rospy.loginfo("Shingles spawned")
  ee0 = 1
  ee1 = 1
  walk1(goal1, 3, 0, ee1)
  send(goal1, client_1)
  ee1 = swapEE(ee1)
  walk1(goal1, 4, 0, ee1)
  ee1 = swapEE(ee1)
  send(goal1, client_1)
  spawnOne(spawn, 0, 1, client_0)

  move0(goal0, 0, 1, ee0)
  send(goal0, client_0)

  pick0(goal0, 0, 1, ee0)
  send(goal0, client_0)

  move0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  place0(goal0, 1, 1, ee0)
  send(goal0, client_0)
  #####################
  walk0(goal0, 1, 0, ee0)
  send(goal0, client_0)
  ee0 = swapEE(ee0)
  ######################
  move0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  pick0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  move0(goal0, 2, 1, ee0)
  send(goal0, client_0)

  place0(goal0, 2, 1, ee0)
  send(goal0, client_0)
  #######################
  # walk0(goal0, 2, 0, ee0)
  # send(goal0, client_0)
  # ee0 = swapEE(ee0)
  # ######################
  # move0(goal0, 2, 1, ee0)
  # send(goal0, client_0)

  # pick0(goal0, 2, 1, ee0)
  # send(goal0, client_0)

  # move0(goal0, 3, 1, ee0)
  # send(goal0, client_0)

  # place0(goal0, 3, 1, ee0)
  # send(goal0, client_0)
  # #######################
  # walk0(goal0, 1, 0, ee0)
  # send(goal0, client_0)
  # ee0 = swapEE(ee0)
  ######################
  walk0(goal0, 0, 0, ee0)
  send(goal0, client_0)
  ee0 = swapEE(ee0)
  ######################
  spawnOne(spawn, 0, 1, client_0)

  move0(goal0, 0, 1, ee0)
  send(goal0, client_0)

  pick0(goal0, 0, 1, ee0)
  send(goal0, client_0)

  move0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  place0(goal0, 1, 1, ee0)
  send(goal0, client_0)
  #####################
  walk0(goal0, 1, 0, ee0)
  send(goal0, client_0)
  ee0 = swapEE(ee0)
  ######################
  move0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  pick0(goal0, 1, 1, ee0)
  send(goal0, client_0)

  move0(goal0, 2, 1, ee0)
  send(goal0, client_0)

  place0(goal0, 2, 1, ee0)
  send(goal0, client_0)


  # walk0(goal0, 1, 0, ee0)
  # walk1(goal1, 3, 0, ee1)
  # # walk1(goal1, 4, 0, 0)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  # walk0(goal0, 2, 1, 0)
  # walk1(goal1, 4, 0, 0)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  # walk0(goal0, 1, 2, 1)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 0, 2, 0)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()

  # walk0(goal0, 0, 1, 1)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 1, 1, 0)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()

  # walk0(goal0, 2, 1, 1)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 2, 2, 0)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()

  # walk0(goal0, 2, 3, 1)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 1, 3, 0)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()

  # walk0(goal0, 0, 2, 1)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 1, 2, 0)
  # client_0.send_goal(goal0)
  # client_0.wait_for_result()
  # walk0(goal0, 1, 1, 0)
  # walk1(goal1, 2, 0, 0)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  # rospy.sleep(1)

  # walk0(goal0, 2, 1, 1)
  # walk1(goal1, 1, 0, 1)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  # rospy.sleep(1)
  
  # walk0(goal0, 2, 0, 0)
  # walk1(goal1, 1, 1, 0)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  # rospy.sleep(1)

  # walk0(goal0, 2, 1, 0)
  # walk1(goal1, 0, 0, 0)
  # client_0.send_goal(goal0)
  # client_1.send_goal(goal1)
  # client_0.wait_for_result()
  # client_1.wait_for_result()

  

  

if __name__ == "__main__":
  main()