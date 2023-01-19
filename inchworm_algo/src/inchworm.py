#!/usr/bin/env python3

from enum import Enum

import cv2 as cv
import numpy as np
from PIL import Image
from inchworm_algo.msg import InchwormMsg
from inchworm_control.msg import InchwormAction, InchwormGoal
from actionlib_msgs.msg import GoalStatus
import rospy, actionlib
import math
from shingle import Shingle, ShingleStatus
from shingle_depot import ShingleDepot
import hex_converter
import random
from std_srvs.srv import Empty
# all x and y are in array coords currently

DEBUG = False

class EEStatus(Enum):
    PLANTED = 0
    IN_AIR = 1


class EEShingleStatus(Enum):
    NO_SHINGLE = 0  # should only be with in_air
    INSTALLED = 1  # should only be with planted
    PLACED = 2  # should only be with planted
    ATTACHED = 3  # should only be with in_air


class EE(Enum):
    BOTTOM_FOOT = 0  
    TOP_FOOT = 1  


class Behavior(Enum):
    SKELETON = 0


class RobotState(Enum):
    PICKUP_SHINGLE_FROM_DEPOT = 0
    MOVE_TO_TARGET = 1
    INSTALL_SHINGLE = 2
    PATROL_FRONTIER = 3
    MOVE_SHINGLE = 4
    MAKE_DECISION = 5
    EXPLORE = 6

class Pattern(Enum):
    OXEN_TURN = 0
    DIAGONAL = 1
    PHYSICS = 2

CONF_CUTOFF_THRESH = 5.5
CONF_EXPLORE_THRESH = 7.0
INSTALLED_CONF = 75.0
DEFAULT_CONF = 10.0

PLACED_SHINGLE_CONF = 16.0
CONF_DECRESS = 0.25
LOW_CONF = 0.6

PROB_OF_EXPLORE_IF_CANT_MOVE = 0.25

# TODO: WE ARE CURRENTLY IGNORRING HALF SHINGLES
class Inchworm():
    id = -1
    bottom_foot_position = [-1, -1]
    top_foot_position = [-1, -1]
    
    

    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    DELAY = 0

    def __init__(self, id=-1, bottom_foot_pos=[-1, -1], top_foot_pos=[-1, -1],
                 bottom_foot_stat=EEStatus.PLANTED, top_foot_stat=EEStatus.PLANTED, bottom_foot_shingle_stat=EEShingleStatus.NO_SHINGLE,
                 top_foot_shingle_stat=EEShingleStatus.NO_SHINGLE, behavior=Behavior.SKELETON, width=1, height=1, shingle_depot_pos=[0], pattern=Pattern.OXEN_TURN, physics=False):
        self.id = id  # this value should not change once it is assigned
        self.bottom_foot_position = bottom_foot_pos
        self.top_foot_position = top_foot_pos
        self.bottom_foot_status = bottom_foot_stat
        self.top_foot_status = top_foot_stat
        self.bottom_foot_shingle_stat = bottom_foot_shingle_stat
        self.top_foot_shingle_stat = top_foot_shingle_stat
        self.behavior = behavior
        self.roof_width = width
        self.roof = [[ShingleStatus.UNINSTALLED for i in range(height)] for j in range(width)]
        for i in range(width):
            self.roof[i][0] = ShingleStatus.INSTALLED
        self.shingle_depot_pos = shingle_depot_pos
        self.next_tick_time = rospy.Time.now()
        self.robot_state = RobotState.MAKE_DECISION
        self.target_bottom_foot_pos = bottom_foot_pos
        self.target_top_foot_pos = bottom_foot_pos
        
        self.foot_shingle_neighbor_to_move_to = 0
        self.ee_shingle_neighbors = []
        self.installing_status = 0

        self.path_for_shingle = []
        if pattern == Pattern.OXEN_TURN.value:
            self.pattern = Pattern.OXEN_TURN
            self.shingle_order = self.create_oxen_turn(width, height)
        elif pattern == Pattern.DIAGONAL.value:
            self.pattern = Pattern.DIAGONAL
            self.shingle_order = self.create_diagonal_order(width, height)
        elif pattern == Pattern.PHYSICS.value:
            self.pattern = Pattern.PHYSICS
            self.shingle_order = self.create_physics_order(width, height)

        self.move_count = 0
        self.state_counts = {"pickup_from_depot": 0, "move_to_target": 0, "install_shingle": 0, "move_shingle": 0, "explore": 0, "move_to_depot": 0}
        self.probe_step = 0
        self.claimed_pos = set()
        self.target = None
        self.roof_confidence = [0.0] * width * height
        for i in range(width):
            self.roof_confidence[i] = INSTALLED_CONF
        self.last_positions = []
        self.stuck_count = 0
        self.old_bottom_foot = None
        self.old_top_foot = None
        self.using_physics = physics
        self.most_recent_target = 0
        if physics:
            #rospy.logwarn(f"waiting for server")
            self.action_client = actionlib.SimpleActionClient(f"/inchworm_action_{self.id}", InchwormAction)
            self.action_client.wait_for_server()
            self.goal_sent_bottom = False
            self.goal_sent_top = False
            self.goal_sent = False
        # while self.top_foot_position != top_foot_pos:
        #     self.move_top_foot(top_foot_pos)
        


    # this is used to create ox-plow order in shingling
    def create_oxen_turn(self, width, height):
        shingle_index_order = []
        if height % 2 == 0:
            for i in range(height ):

                if i % 2 == 0:

                    for j in range(width):
                        shingle_index_order.append([j, i])
                else:
                    for j in range(1, width + 1):
                        shingle_index_order.append([width - j, i])
        else:
            for i in range(height ):

                if i % 2 == 0:

                    for j in range(1, width + 1):
                        shingle_index_order.append([width - j, i])
                else:
                    for j in range(width):
                        shingle_index_order.append([j, i])

        return shingle_index_order

    def create_diagonal_order(self, width, height):
        in_bounds = lambda x,y: x >= 0 and x < width and y >= 0 and y < height
        #rospy.logwarn(f"using diagonal_order")
        shingle_index_order = []

        UP_RIGHT_EVEN = [1, 1]
        UP_RIGHT_ODD  = [0, 1]

        for i in range(width):
            shingle_index_order.append([i, 0])

        # Iterate right to left
        for col in range(width, -height, -1):
            start = [col, 1]

            if in_bounds(start[0], start[1]):
                shingle_index_order.append(start)

            next = start
            for row in range(1, height):
                if row % 2 == 0:
                    # even
                    next = [next[0] + UP_RIGHT_EVEN[0], next[1] + UP_RIGHT_EVEN[1]]
                else:
                    # odd
                    next = [next[0] + UP_RIGHT_ODD[0], next[1] + UP_RIGHT_ODD[1]]

                if in_bounds(next[0], next[1]):
                    shingle_index_order.append(next)

        # rospy.logwarn(f"length of shingle order {len(shingle_index_order)}")
        # print(shingle_index_order)
        
        return shingle_index_order

    # Physics sim manual order, to show we can do it in the algo sim too
    def create_physics_order(_, width, height):
        shingle_index_order = []

        for row in range(height):
            for col in range(width-1, -1, -1):
                shingle_index_order.append([col, row])

        return shingle_index_order

    def create_from_message(self, msg):
        self.id = msg.id
        self.bottom_foot_position = msg.bottom_foot_pos
        self.top_foot_position = msg.top_foot_pos
        self.bottom_foot_status = msg.bottom_foot_status
        self.top_foot_status = msg.top_foot_status
        self.bottom_foot_shingle_pos = msg.bottom_foot_shingle_pos
        self.top_foot_position = msg.top_foot_position
        self.behavior = msg.behavior
        return self

    def get_shingle_conf(self, x, y):
        return self.roof_confidence[x + y * self.roof_width]

    def set_shingle_conf(self, x, y, shingle_state):
        self.roof_confidence[x + y * self.roof_width] = shingle_state


    def get_shingle_state(self, x, y):
        if self.get_shingle_conf(x, y) <= CONF_CUTOFF_THRESH:
            self.set_shingle_state(x, y, ShingleStatus.UNINSTALLED)
        return self.roof[x][y]


    def set_shingle_state(self, x, y, shingle_state, realiable=True):
        if x >= 0 and y >= 0:
            if realiable:
                if shingle_state == ShingleStatus.INSTALLED:
                    self.set_shingle_conf(x, y, INSTALLED_CONF)
                # elif shingle_state == ShingleStatus.PLACED:
                #     self.set_shingle_conf(x, y, PLACED_SHINGLE_CONF)
                else:
                    self.set_shingle_conf(x, y, DEFAULT_CONF)
            # elif shingle_state != self.roof[x][y] and shingle_state == ShingleStatus.PLACED:
            #     self.set_shingle_conf(x, y, PLACED_SHINGLE_CONF)
            
            if self.roof[x][y] != ShingleStatus.INSTALLED:
                self.roof[x][y] = shingle_state


    def place_shingle(self, ee, shingle, roof):
        if self.using_physics:
            if not self.goal_sent:
                goal = InchwormGoal()
                goal.action_type = 2
                if ee == EE.BOTTOM_FOOT:
                    goal.coord_x = self.bottom_foot_position[0]
                    goal.coord_y = self.bottom_foot_position[1]
                else:
                    goal.coord_x = self.top_foot_position[0]
                    goal.coord_y = self.top_foot_position[1]
                goal.end_effector = ee.value
                self.action_client.send_goal(goal)
                self.goal_sent = True
            else:
                if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                    self.goal_sent = False
                    if (ee == EE.BOTTOM_FOOT):
                        # rospy.loginfo(f" bottom foot position {[self.bottom_foot_position[0], self.bottom_foot_position[1]]}")
                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.bottom_foot_position[0], self.bottom_foot_position[1])
                            
                        roof.place_shingle(shingle, self.bottom_foot_position)
                        self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.PLACED)
                        self.bottom_foot_shingle_stat = EEShingleStatus.PLACED
                        
                        # updates newly placed shingle
                        self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
                        # updates the shingle that the inchworm is plated on
                        self.update_shingle_with_current_roof(roof, self.top_foot_position)
                    elif(ee == EE.TOP_FOOT):
                        # rospy.loginfo(f" top foot position {[self.top_foot_position[0], self.top_foot_position[1]]}")

                        self.shingle_to_move = self.shingle_to_move.place_shingle(
                            self.top_foot_position[0], self.top_foot_position[1])

                        roof.place_shingle(self.shingle_to_move, self.top_foot_position)

                        self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.PLACED)
                        self.top_foot_shingle_stat = EEShingleStatus.PLACED
                        # updates newly placed shingle
                        self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
                        # updates the shingle that the inchworm is plated on
                        self.update_shingle_with_current_roof(roof, self.bottom_foot_position)
                    else:
                        #rospy.logwarn(f"ATTEMPTED TO MOVE A THIRD ({ee}) LEG, BAD")
                        pass


        else:

            # rospy.loginfo(f"using the {ee} foot to place a shingle {[shingle.x_coord, shingle.y_coord]} at:")
            if (ee == EE.BOTTOM_FOOT):
                # rospy.loginfo(f" bottom foot position {[self.bottom_foot_position[0], self.bottom_foot_position[1]]}")
                self.shingle_to_move = self.shingle_to_move.place_shingle(
                    self.bottom_foot_position[0], self.bottom_foot_position[1])
                    
                roof.place_shingle(shingle, self.bottom_foot_position)
                self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.PLACED)
                self.bottom_foot_shingle_stat = EEShingleStatus.PLACED
                
                # updates newly placed shingle
                self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
                # updates the shingle that the inchworm is plated on
                self.update_shingle_with_current_roof(roof, self.top_foot_position)
            elif(ee == EE.TOP_FOOT):
                # rospy.loginfo(f" top foot position {[self.top_foot_position[0], self.top_foot_position[1]]}")

                self.shingle_to_move = self.shingle_to_move.place_shingle(
                    self.top_foot_position[0], self.top_foot_position[1])

                roof.place_shingle(self.shingle_to_move, self.top_foot_position)

                self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.PLACED)
                self.top_foot_shingle_stat = EEShingleStatus.PLACED
                # updates newly placed shingle
                self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
                # updates the shingle that the inchworm is plated on
                self.update_shingle_with_current_roof(roof, self.bottom_foot_position)
            else:
                #rospy.logwarn(f"ATTEMPTED TO MOVE A THIRD ({ee}) LEG, BAD")
                pass
        


    def pickup_shingle(self, ee, shingle, roof):
        if self.using_physics:
            if not self.goal_sent:
                goal = InchwormGoal()
                goal.action_type = 1
                goal.coord_x = shingle.x_coord
                goal.coord_y = shingle.y_coord
                goal.end_effector = ee.value
                self.action_client.send_goal(goal)
                self.goal_sent = True
            else:
                if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                    shingle = roof.pickup_shingle([shingle.x_coord, shingle.y_coord])

                    if ee == EE.BOTTOM_FOOT:
                        self.set_shingle_state(self.bottom_foot_position[0], self.bottom_foot_position[1], ShingleStatus.UNINSTALLED)

                        self.bottom_foot_shingle_stat = EEShingleStatus.ATTACHED
                        
                        # updates the shingle that the inchworm is plated on
                        self.update_shingle_with_current_roof(roof, self.top_foot_position)
                    else:
                        self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.UNINSTALLED)

                        self.top_foot_shingle_stat = EEShingleStatus.ATTACHED

                        # updates the shingle that the inchworm is plated on
                        self.update_shingle_with_current_roof(roof, self.bottom_foot_position)
                    self.shingle_to_move = shingle
                    self.goal_sent = False
        else:
            shingle = roof.pickup_shingle([shingle.x_coord, shingle.y_coord])
            if (ee == EE.BOTTOM_FOOT):
                self.set_shingle_state(self.bottom_foot_position[0], self.bottom_foot_position[1], ShingleStatus.UNINSTALLED)

                self.bottom_foot_shingle_stat = EEShingleStatus.ATTACHED
                
                # updates the shingle that the inchworm is plated on
                self.update_shingle_with_current_roof(roof, self.top_foot_position)
            else:

                self.set_shingle_state(self.top_foot_position[0], self.top_foot_position[1], ShingleStatus.UNINSTALLED)

                self.top_foot_shingle_stat = EEShingleStatus.ATTACHED

                # updates the shingle that the inchworm is plated on
                self.update_shingle_with_current_roof(roof, self.bottom_foot_position)

            
            roof.pickup_shingle([shingle.x_coord, shingle.y_coord])
            #rospy.logwarn(f"inchworm {self.id} is picking up a shingle")
            self.shingle_to_move = shingle




    def install_shingle(self, ee, shingle, roof):
        # figure out x and y -> going off bottom_foot?
        if (ee == EE.BOTTOM_FOOT):
            roof.install_shingle(shingle)
            self.bottom_foot_shingle_stat = EEShingleStatus.INSTALLED
            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.INSTALLED)
            
            # updates newly installed shingle
            self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
            # updates the shingle that the inchworm is plated on
            self.update_shingle_with_current_roof(roof, self.top_foot_position)



        else:
            roof.install_shingle(shingle)
            self.top_foot_shingle_stat = EEShingleStatus.INSTALLED

            self.set_shingle_state(shingle.x_coord, shingle.y_coord, ShingleStatus.INSTALLED)

            # updates newly installed shingle
            self.update_shingle_with_current_roof(roof, [shingle.x_coord, shingle.y_coord])
            # updates the shingle that the inchworm is plated on
            self.update_shingle_with_current_roof(roof, self.bottom_foot_position)

        # # translate to shingle status
        # if  > 3:
        #     stat = 0
        # shingle.shingle_status = stat



    

    def read_shingle_at(self, real_roof, shingle_coord):
        shingle = real_roof.get_shingle(shingle_coord[0], shingle_coord[1])

        # if shingle is not None:

        # update the "roof" to include shingle, if the shingle is in a place
        self.set_shingle_state(shingle_coord[0], shingle_coord[1], shingle.shingle_status) 
        #rospy.logwarn(f"inchworm {self.id} reading {shingle.most_recent_target} -------------------------------")
        if shingle.most_recent_target > self.most_recent_target:
            rospy.logwarn(f"inchworm {self.id} found a recent target of {shingle.most_recent_target}, which is less that {self.most_recent_target}")
            for i in range(self.most_recent_target, shingle.most_recent_target):
                self.set_shingle_state(self.shingle_order[i][0], self.shingle_order[i][1], ShingleStatus.INSTALLED)
            self.most_recent_target = shingle.most_recent_target
            
            # self.rebuild_roof()
        if shingle.shingle_status == ShingleStatus.INSTALLED:
            neighbor_status = shingle.get_neighbor_locations_and_status()
            i = 0
            for key, value in neighbor_status.items():
                
                # rospy.loginfo(f"inchworm {self.id} updating {key} with {value} on shingle {shingle_coord} using {key}, {neighbor_status}")
                # rospy.logwarn(self.roof)
                if (key[0] > -1 and key[0] < len(self.roof) and (key[1] > -1 and key[1] < len(self.roof[0]))):
                        # if value == ShingleStatus.INSTALLED:
                        #     self.set_shingle_state(key[0], key[1], ShingleStatus.INSTALLED, realiable=True)
                        if value == ShingleStatus.PLACED:
                            if self.get_shingle_state(key[0], key[1]) == ShingleStatus.INSTALLED:
                                new_shingle_status = ShingleStatus.INSTALLED
                            else:
                                new_shingle_status = ShingleStatus.PLACED
                                self.set_shingle_state(key[0], key[1], ShingleStatus.PLACED, realiable=True)
                            neighbor_index = shingle.convert_to_neighbor_index(key)
                            try:
                                shingle.update_neighbor(neighbor_index, new_shingle_status)
                            except Exception:
                                pass
        # self.update_shingle_with_current_roof(real_roof, shingle_coord)
        # else:
        #     self.set_shingle_state(shingle_coord[0], shingle_coord[1], ShingleStatus.UNINSTALLED)
        #     #rospy.logwarn(f"inchworm {self.id} has a plated foot on an empty position")
            





    def update_shingle_with_current_roof(self, real_roof, shingle_pos):
        shingle = real_roof.get_shingle(shingle_pos[0], shingle_pos[1])
        if self.most_recent_target > shingle.most_recent_target:
            shingle.most_recent_target = self.most_recent_target
        if shingle.y_coord % 2 == 0:
            realative_neighbors = self.EVEN_ROW_N_LOOKUP
        else:
            realative_neighbors = self.ODD_ROW_N_LOOKUP
        for i, v in enumerate(realative_neighbors):
            neighbor_coord = [shingle.x_coord + v[0], shingle.y_coord + v[1]]
            if (neighbor_coord[0] > -1 and neighbor_coord[0] < self.roof_width) and (neighbor_coord[1] > -1 and neighbor_coord[1] < len(self.roof[0])):
                
                shingle.update_neighbor(i, self.get_shingle_state(neighbor_coord[0], neighbor_coord[1]))
        real_roof.set_shingle(shingle_pos[0], shingle_pos[1], shingle)
        


    def move_bottom_foot(self, new_pos):
        
        if self.check_self_claimed(new_pos):
            if self.using_physics:
                if not self.goal_sent_bottom:
                    goal = InchwormGoal()
                    goal.action_type = 0
                    goal.coord_x = new_pos[0]
                    goal.coord_y = new_pos[1]
                    goal.end_effector = False
                    self.action_client.send_goal(goal)
                    self.goal_sent_bottom = True
                    self.bottom_foot_status = EEStatus.IN_AIR

                else:
                    if self.action_client.get_state() != GoalStatus.ACTIVE and self.action_client.get_state() != GoalStatus.PENDING or self.action_client.get_state() == GoalStatus.SUCCEEDED:
                        self.bottom_foot_position = new_pos
                        self.goal_sent_bottom = False
            else:
                self.bottom_foot_status = EEStatus.IN_AIR
                self.bottom_foot_position = new_pos
            self.move_count += 1

    def move_top_foot(self, new_pos):
        
        if self.check_self_claimed(new_pos):
            if self.using_physics:
                if not self.goal_sent_top:
                    goal = InchwormGoal()
                    goal.action_type = 0
                    goal.coord_x = new_pos[0]
                    goal.coord_y = new_pos[1]
                    goal.end_effector = True
                    self.action_client.send_goal(goal)
                    self.goal_sent_top = True
                    self.top_foot_status = EEStatus.IN_AIR
                    #rospy.logwarn(f"inchworm {self.id} sending top foot to {new_pos}")
                else:
                    if self.action_client.get_state() != GoalStatus.ACTIVE and self.action_client.get_state() != GoalStatus.PENDING or self.action_client.get_state() == GoalStatus.SUCCEEDED:
                        self.top_foot_position = new_pos
                        self.goal_sent_top = False
                        #rospy.logwarn(f"inchworm {self.id} has made it to the target {new_pos}")
            else:
                self.top_foot_status = EEStatus.IN_AIR
                self.top_foot_position = new_pos
            self.move_count += 1
            


    def calc_inchworm_pos(self):
        '''calculates the effective position of the inchworm'''
        if self.target[1] == (self.bottom_foot_position[1] + self.top_foot_position[1]) /2:
            inchworm_pos = [(self.bottom_foot_position[0] + self.top_foot_position[0]) / 2,
                                    (self.bottom_foot_position[1] + self.top_foot_position[1]) / 2]
        elif ((self.bottom_foot_position[1] + self.top_foot_position[1]) /2)%1 != 0:
            # special case if the robot is on a diagonal
            bottom_foot_dis_to_target = Inchworm.dist(self.bottom_foot_position, self.target)
            top_foot_dis_to_target = Inchworm.dist(self.top_foot_position, self.target)
            if top_foot_dis_to_target > bottom_foot_dis_to_target:
                 inchworm_pos = self.bottom_foot_position
            else:
                inchworm_pos = self.top_foot_position
                
        else:
            inchworm_pos = [max(self.bottom_foot_position[0], self.top_foot_position[0]),
                                    max(self.bottom_foot_position[1], self.top_foot_position[1])]

        return inchworm_pos
    
    def rebuild_roof(self):  # TODO: IF THIS STARTS TO MAKE THINGS SLOW, MAKE IT NOT RECUSIVE
        for i, row in enumerate(self.roof):
            for j, occ in enumerate(row):
                # rospy.loginfo(f"inchworm {self.id} rebuilding {[j, i]}: {occ}")
                if occ == ShingleStatus.INSTALLED:
                    if self.get_shingle_conf(i, j) > CONF_CUTOFF_THRESH:
                        self.make_children_valid(i, j)
                        


    def make_children_valid(self, x, y):
        self.set_shingle_state(x, y, ShingleStatus.INSTALLED, realiable=True)
        if y % 2 == 0:  # handle an even row
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_x < len(self.roof) - 1 and test_y > -1 and test_y < len(self.roof[0]) - 1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_x < len(self.roof) - 1 and test_y > -1 and test_y < len(self.roof[0]) - 1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
        else:
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_x < len(self.roof) -1 and test_y > -1 and test_y < len(self.roof[0]) - 1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)
            test_x = x + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = y + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_x < len(self.roof) - 1 and test_y > -1 and test_y < len(self.roof[0]) - 1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.UNINSTALLED:
                self.make_children_valid(test_x, test_y)


    def claim_pos(self, real_roof, position):
        self.claimed_pos.add(tuple(position))
        real_roof.claim_position(position)
  

    def unclaim_pos(self, real_roof, position):
        self.claimed_pos.remove(tuple(position))
        real_roof.unclaim_position(position)
        

    def check_self_claimed(self, position):
        return tuple(position) in self.claimed_pos

    def valid_uninstalled_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors where the inchworm can and there are no shingles'''
        # rospy.loginfo(f"valid neighbors of the {foot_to_move} are {neighbors}")
        move_targets = []
        for n in neighbors:
            # rospy.loginfo(f"The status of neighbor {n} is {self.get_shingle_state(n[0], n[1])}")
            if self.get_shingle_state(n[0], n[1]) != ShingleStatus.INSTALLED and (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]])):
                move_targets.append({"pos": n, "foot": foot_to_move})
        return move_targets

    def valid_installed_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors where the inchworm can move where the shingles are installed'''
        move_targets = []
        for n in neighbors:
            if (((self.get_shingle_state(n[0], n[1]) == ShingleStatus.INSTALLED and self.get_shingle_conf(n[0], n[1]) > 1.5)) and 
                (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]]))):
                move_targets.append({"pos": n, "foot": foot_to_move})
        return move_targets

    def valid_foot_positions(self, neighbors, foot_to_move, real_roof):
        '''returns all neighbors that it can move a foot to'''
        move_targets = []
        for n in neighbors:
            if (real_roof.get_occ_position([n[0], n[1]]) == 0 or self.check_self_claimed([n[0], n[1]])):
                move_targets.append({"pos": n, "foot": foot_to_move})
        random.shuffle(move_targets)
        return move_targets


    def get_best_placed_shingle(self, shingles):
        neighbors = self.get_shingle_neighbors(self.bottom_foot_position)
        neighbors.extend(self.get_shingle_neighbors(
            self.top_foot_position))
        neighbors.sort(key=lambda x: Inchworm.dist(x, self.target))
        shingle_to_move = neighbors[0]
        #rospy.loginfo(f"shingles to move {shingle_to_move}")
        shingle = Shingle()
        shingle.shingle_status = ShingleStatus.PLACED
        shingle.x_coord = shingle_to_move[0]
        shingle.y_coord = shingle_to_move[1]
        return shingle
        
    def next_to_placed_shingle(self, pos):
        neighbors = self.get_shingle_pos_neighbors(pos)

        for n in neighbors:

            if n[0] < self.roof_width:
                read_shingle = self.get_shingle_state(n[0], n[1])

                if read_shingle is not None:
                    if read_shingle == ShingleStatus.PLACED:
                        return True
        return False

    def placed_shingle_is_valid(self, shingle_coord):
        '''checks to make sure a shingle location is valid for install'''
        validity_count = 0
        if shingle_coord[1] == 0:
            validity_count += 2

        if shingle_coord[1] % 2 == 0:
            test_x = shingle_coord[0] + Inchworm.EVEN_ROW_N_LOOKUP[1][0]
            test_y = shingle_coord[1] + Inchworm.EVEN_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and test_x < self.roof_width and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            test_x = shingle_coord[0] + Inchworm.EVEN_ROW_N_LOOKUP[2][0]
            test_y = shingle_coord[1] + Inchworm.EVEN_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_y > -1 and test_x < self.roof_width and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            if shingle_coord[0] == self.roof_width - 1:
                validity_count += 1
            
        else:
            test_x = shingle_coord[0] + Inchworm.ODD_ROW_N_LOOKUP[1][0]
            test_y = shingle_coord[1] + Inchworm.ODD_ROW_N_LOOKUP[1][1]
            if test_x > -1 and test_y > -1 and test_x < self.roof_width and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            test_x = shingle_coord[0] + Inchworm.ODD_ROW_N_LOOKUP[2][0]
            test_y = shingle_coord[1] + Inchworm.ODD_ROW_N_LOOKUP[2][1]
            if test_x > -1 and test_x < self.roof_width and test_y > -1 and self.get_shingle_state(test_x, test_y) == ShingleStatus.INSTALLED:
                validity_count += 1
            if shingle_coord[0] == 0:
                validity_count += 1

        return validity_count >= 2

    # TODO: this will be more complicated in the future - could encode behaviors in here
    def choose_shingle_target(self):
        '''chooses the target install location'''
        x_coord = -1
        y_coord = -1

        # rospy.loginfo(f"inchworm {self.id} has a shingle order of {self.shingle_order}")
        # for i in reversed(range(len(self.roof[0]))):
        #     row = []
        #     for j in range(len(self.roof)):
        #         row.append(self.roof[j][i].value)
        #     #rospy.logwarn(f"inchworm {self.id} row {i} has values {row}")
        for i, coord in enumerate(self.shingle_order):
            val = self.get_shingle_state(coord[0], coord[1])
            # rospy.loginfo(val)
            if (val != ShingleStatus.INSTALLED and i - 1 > -1 and 
                ((self.get_shingle_state(self.shingle_order[i - 1][0], self.shingle_order[i - 1][1]) == ShingleStatus.INSTALLED))):
                # rospy.loginfo(f"inchworm {self.id} has possible target {coord}")
                if self.placed_shingle_is_valid(coord) == True:
                    x_coord = coord[0]
                    y_coord = coord[1]
                    if self.most_recent_target < i:
                        self.most_recent_target = i
                # else:
                #     self.set_shingle_state(coord[0], coord[1], ShingleStatus.INSTALLED)
                #     self.rebuild_roof()
                    # return coord
                    # if self.pattern == Pattern.DIAGONAL:
                    #     return [x_coord, y_coord]
                # rospy.loginfo(
                #     f"inchworm {self.id} setting tile {coord} as target")
                # if y_coord % 2 == 0:
                #     break
                # else:
                #     if x_coord + 1 < self.roof_width and self.roof[i + 1] == 2:
                #         break
                #     elif x_coord == self.roof_width -1:
                #         break
        # rospy.loginfo(f"inchworm {self.id} has a target of {[x_coord, y_coord]} from choose target {self.shingle_order}")
        # for i in reversed(range(len(self.roof[0]))):
        #     row = []
        #     for j in range(len(self.roof)):
        #         row.append(self.roof[j][i].value)
        #     #rospy.logwarn(f"inchworm {self.id} row {i} has values {row}")

        return [x_coord, y_coord]

    def dist(ee_pos, target_pos):
        return hex_converter.evenr_distance(ee_pos, target_pos)
        # return math.sqrt((ee_pos[0] - target_pos[0])**2 + (ee_pos[1] - target_pos[1])**2)

    def next_to_shingle_depot(self, shingle_depot_location):
        '''checks to see if the inchworm has either foot next to the shingle depot'''
        #rospy.loginfo(f"bottom_foot at {self.bottom_foot_position}")
        #rospy.loginfo(f"top_foot at {self.top_foot_position}")
        if self.bottom_foot_position[0] == 0:
            if self.bottom_foot_position[1] == shingle_depot_location:
                return True
            elif self.bottom_foot_position[1] > shingle_depot_location:
                self.shingle_depot_pos[0] = shingle_depot_location
        if self.top_foot_position[0] == 0:

            if self.top_foot_position[1] == shingle_depot_location:
                return True
            elif self.top_foot_position[1] > shingle_depot_location:
                self.shingle_depot_pos[0] = shingle_depot_location

        return False

    def get_shingle_pos_neighbors(self, foot_pos):
        '''gets all foot neighbors that exist on the roof'''
        neighbor_pos = []
        if foot_pos[1] % 2 == 0:  # even row lookup
            for n in Inchworm.EVEN_ROW_N_LOOKUP:
                new_neighbor_pos = [foot_pos[0] + n[0], foot_pos[1] + n[1]]
                if new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof[0]) and new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(
                            [new_neighbor_pos[0], new_neighbor_pos[1]])
        else:
            for n in Inchworm.ODD_ROW_N_LOOKUP:
                new_neighbor_pos = [foot_pos[0] + n[0], foot_pos[1] + n[1]]
                if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof[0]) and new_neighbor_pos != self.bottom_foot_position and new_neighbor_pos != self.top_foot_position:
                    if new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                        neighbor_pos.append(new_neighbor_pos)
        return neighbor_pos


    def set_least_conf_neighbor_as_target(self):


        neighbors = self.get_shingle_pos_neighbors(self.bottom_foot_position)
        neighbors.extend(self.get_shingle_pos_neighbors(self.top_foot_position))
        conf = []
        state = []
        for n in neighbors:
            conf.append(self.get_shingle_conf(n[0], n[1]))
            state.append(self.get_shingle_state(n[0], n[1]).value)

        possible_targets = list(zip(neighbors, conf, state))

        possible_targets = list(filter(lambda x: x[1] < CONF_EXPLORE_THRESH, possible_targets))

        random.shuffle(possible_targets)

        possible_targets.sort(key=lambda x: (-x[2], x[1]))

        # rospy.loginfo(f"inchworm {self.id} has possible explore targets{possible_targets}")
        if len(possible_targets) > 0:
            self.target = possible_targets[0][0]
        else:
            roof_image = []
            for i in range(len(self.roof[0])):
                row = []
                for j in range(len(self.roof)):
                    row.append((self.roof[j][i].value))
                roof_image.append(row)

            roof_image = np.asarray(roof_image)


            # convert image to grayscale image
            roof_image = np.array(roof_image.astype(np.uint8))
            # gray_image = cv.cvtColor(np.int8(roof_image), cv.COLOR_BGR2GRAY)


            

            # convert the grayscale image to binary image
            ret,thresh = cv.threshold(roof_image,1,255,1)

            # calculate moments of binary image
            M = cv.moments(thresh)

            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"]) + random.randint(-int(self.roof_width/4), int(self.roof_width/4))
            cY = int(M["m01"] / M["m00"]) + random.randint(-int(self.roof_width/4), int(self.roof_width/4))


            #rospy.logwarn(f"inchworm {self.id} is exploring to a random location centroid at {[cX, cY]}")
            self.target = [cX, cY]

    def make_state_explore(self, real_roof):
        
        self.set_least_conf_neighbor_as_target()

        if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_foot_positions, real_roof)
        else:
            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_foot_positions, real_roof)
        if len(self.ee_shingle_neighbors) > 0 and (real_roof.get_occ_position(self.ee_shingle_neighbors[0]["pos"]) == 0 or self.check_self_claimed(self.ee_shingle_neighbors[0]["pos"])):
            self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
            self.explore_state = 1
            #rospy.loginfo(f'inchworm {self.id} is exploring {self.ee_shingle_neighbors[0]["pos"]}')
            self.state_counts["explore"] += 1
            self.robot_state = RobotState.EXPLORE
        else:
            self.robot_state = RobotState.MAKE_DECISION
      



    def make_state_move_to_depot(self, real_roof):
        '''makes the state move to depot'''
        self.target = [0, real_roof.get_shingle_depot_location(False)]
        # rospy.loginfo(f"inchworm {self.id} going to depot")
        # figures out the position to move toward the shingle depot
        self.stuck_count = 0 
        if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
        else:
            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
        # rospy.logwarn(f"inchworm {self.id}: {self.last_positions}")
        # rospy.logwarn(f"inchworm {self.id} at {self.calc_inchworm_pos()}")
        if len(self.ee_shingle_neighbors) > 0 and self.calc_inchworm_pos() not in self.last_positions and (real_roof.get_occ_position(self.ee_shingle_neighbors[0]["pos"]) == 0 or self.check_self_claimed(self.ee_shingle_neighbors[0]["pos"])):
            self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
            self.state_counts["move_to_depot"] += 1
            self.robot_state = RobotState.MOVE_TO_TARGET
        elif random.random() < PROB_OF_EXPLORE_IF_CANT_MOVE:
            self.make_state_explore(real_roof)

    def get_shingle_neighbors(self, pos):
        '''gets all of the neighbors of a shingle position where the shingle is placed'''
        neighbors = self.get_shingle_pos_neighbors(pos)
        shingle_neighbors = []
        for n in neighbors:

            if n[1] < len(self.roof[0]) and n[0] < self.roof_width:
                read_shingle = self.get_shingle_state(n[0], n[1])

                if read_shingle is not None:
                    if read_shingle == ShingleStatus.PLACED:
                        shingle_neighbors.append(n)
        return shingle_neighbors



    def decress_conf(self):
        for i, val in enumerate(self.roof_confidence):
                if val > LOW_CONF:
                    self.roof_confidence[i] = val - CONF_DECRESS


    def make_decision(self, real_roof):
        # rospy.loginfo(f"robot {self.id} has claimed tiles {self.claimed_pos}")
        # rospy.loginfo(f"robot {self.id} is in state {self.robot_state}")

        # general idea is that this contitional is run everytime the robot has to make a desision,
        
        if not self.using_physics:
            self.decress_conf()

        if self.robot_state == RobotState.MAKE_DECISION:
            # read the shingles at the current foot positions
            # I am assuming that everytime this loop is run, both feet will be on the ground and we will want to read both shingles
            # TODO: allow inchworms to read the full data from a shingle and rebuild based off of that
            #   if the data read does not match the inchworms such that the shingles are out of date, update the shingle
            
            if self.using_physics:
                self.decress_conf()

            # TODO: change this, will require a working probing state and being able to read shingles
            # self.roof = []
            # for row in real_roof.shingle_array:
            #     for element in row:
            #         if element is not None:
            #             self.roof.append(element.shingle_status)
            #         else:
            #             self.roof.append(ShingleStatus.UNINSTALLED)
            for pos in self.claimed_pos.copy():
                self.unclaim_pos(real_roof, pos)

            
            self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
            self.update_shingle_with_current_roof(real_roof, self.top_foot_position)

            # rospy.loginfo(f"the roof after rebuild is {self.roof}")

            self.claim_pos(real_roof,self.bottom_foot_position)
            self.claim_pos(real_roof,self.top_foot_position)

            # rebuild the roof based on constraints
            self.rebuild_roof()

            installing = False

            if self.target == None:
                self.target = [0, self.shingle_depot_pos[0]]
            self.last_positions.append(self.calc_inchworm_pos())
            if len(self.last_positions) > 4:
                self.last_positions = []

            # rospy.loginfo(f"inchworm {self.id} bottom_foot is next to placed shingles : {self.next_to_placed_shingle(self.bottom_foot_position, shingles)}")
            # rospy.loginfo(f"inchworm {self.id} top_foot is next to placed shingles : {self.next_to_placed_shingle(self.top_foot_position, shingles)}")


            # check either of the end effectors are next is next to a placed shingle
            if self.next_to_placed_shingle(self.bottom_foot_position) or self.next_to_placed_shingle(self.top_foot_position):
               #rospy.loginfo(f"inchworm {self.id} is next to a placed shingle")
                
                self.target = self.choose_shingle_target()

               #rospy.loginfo(f"inchworm {self.id} has a target of {self.target}")

                # get the placed shingles  
                placed_shingle = self.get_best_placed_shingle(real_roof.shingle_array)
                

                inchworm_pos = self.calc_inchworm_pos()

                self.path_for_shingle = self.tile_path([placed_shingle.x_coord, placed_shingle.y_coord], self.target)
                
                if len(self.path_for_shingle) > 1:
                   #rospy.loginfo(f"inchworm {self.id} has a path of {self.path_for_shingle}")
                    self.target = self.path_for_shingle[1]

                # rospy.loginfo(f"inchworm {self.id} set target at {self.target}")

                shingle_moving_position = self.inchworm_spot_to_move_shingle(self.path_for_shingle[0], self.target)
                # rospy.loginfo(f"the spot to move a shingle is {shingle_moving_position}")

                # check if placed shingle is in the target position and should be installed
                if len(self.path_for_shingle) == 1:
                    if real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0 or self.check_self_claimed([placed_shingle.x_coord, placed_shingle.y_coord]):
                        self.install_shingle_target = placed_shingle
                        self.state_counts["install_shingle"] += 1
                        self.robot_state = RobotState.INSTALL_SHINGLE
                        # signal intention to move
                        self.claim_pos(real_roof, [placed_shingle.x_coord, placed_shingle.y_coord]) 
                        # rospy.logwarn(
                        #     f"inchworm {self.id} installing shingle")
                        installing = True
                        self.installing_status = 1
                    else:
                        self.make_state_move_to_depot(real_roof)


                # Check if either of the feet are on the installed neighbor of both the shingle and the next point in the path, if they are not, move there
                elif not(self.top_foot_position[0] == shingle_moving_position[0] and self.top_foot_position[1] == shingle_moving_position[1]) and not(
                    self.bottom_foot_position[0] == shingle_moving_position[0] and self.bottom_foot_position[1] == shingle_moving_position[1]):
                    # when you are in here, the inchworm is moving along installed shingles, or installing a shingle

                    # rospy.loginfo(f"incworm {self.id} wants to move, bottom foot at {self.bottom_foot_position}, top foot at {self.top_foot_position}, moving space is {shingle_moving_position}")
                   
                    # Check if the top foot is next to the moving spot, if it is move the bottom foot otherwise move the top foot
                    if (self.is_neighbor(self.top_foot_position, shingle_moving_position)):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
                    elif(self.is_neighbor(self.bottom_foot_position, shingle_moving_position)):
                         self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)

                    # if neither foot is next to where it can move the shingle use the distance check (happens when the shingle needs to move to a row 2 rows above the inchworm)
                    else:
                        if Inchworm.dist(self.top_foot_position, shingle_moving_position) > Inchworm.dist(self.bottom_foot_position, shingle_moving_position):
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
                        else:
                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)


                    # if the inchworm is not installing a shingle, initate movement of the inchworm to the target
                    if not installing:
                        # checks to make sure that ee_shingle_neighbors has a valid movement option
                        if (len(self.ee_shingle_neighbors) > 0 and 
                            self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED):
                            # rospy.loginfo(f"inchworm {self.id} is claiming {self.ee_shingle_neighbors[0]['pos']} and initate move")
                            self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                            self.state_counts["move_to_target"] += 1
                            self.robot_state = RobotState.MOVE_TO_TARGET
                        
                else:  # The inchworm needs to move a shingle
                    # move a shingle closer to the target
                    # rospy.loginfo(f"inchworm {self.id} wants to move a shingle closer to the target")
                    self.shingle_to_move = placed_shingle
                    placed_shingle_location = [
                        placed_shingle.x_coord, placed_shingle.y_coord]
                    # this gets all the neighbors for each foot, and sorts them based on distance to the target
                    top_foot_shingle_neighbors = self.valid_uninstalled_foot_positions(
                        self.get_shingle_pos_neighbors(self.top_foot_position), EE.BOTTOM_FOOT, real_roof)
                    bottom_foot_shingle_neighbors = self.valid_uninstalled_foot_positions(
                        self.get_shingle_pos_neighbors(self.bottom_foot_position), EE.TOP_FOOT, real_roof)
                    top_foot_shingle_neighbors.sort(
                        key=lambda x: Inchworm.dist(x["pos"], self.target))
                    bottom_foot_shingle_neighbors.sort(
                        key=lambda x: Inchworm.dist(x["pos"], self.target))
                    
                    # rospy.loginfo(
                    #     f"placed shingle location {placed_shingle_location}")
                    # rospy.loginfo(
                    #     f"inchworm {self.id} bottom_foot dist to placed shingle :{Inchworm.dist(placed_shingle_location, self.bottom_foot_position)}")
                    # rospy.loginfo(
                    #      f"bottom_foot shingle neighbors: {bottom_foot_shingle_neighbors}")
                    # rospy.loginfo(
                    #     f"inchworm {self.id} top_foot dist to placed shingle :{Inchworm.dist(placed_shingle_location, self.top_foot_position)}")
                    # rospy.loginfo(
                    #      f"top_foot shingle neighbors: {top_foot_shingle_neighbors}")
                    # strip the neighbor options from the dicts to more easly compair them all
                    top_foot_neighbors = [
                        value for elem in top_foot_shingle_neighbors for value in elem.values()]
                    bottom_foot_neighbors = [
                        value for elem in bottom_foot_shingle_neighbors for value in elem.values()]
                    claimed_new_postion = False
                    # if the placed shingle is in one neighbor list and not the other, move the foot that does not have the shingle in the list
                    if placed_shingle_location in top_foot_neighbors and placed_shingle_location not in bottom_foot_neighbors:
                        # move the bottom foot to the placed shingle
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                        self.claim_pos(real_roof, top_foot_shingle_neighbors[0]["pos"])
                        claimed_new_postion = True
                        self.old_bottom_foot = self.bottom_foot_position
                    elif placed_shingle_location in bottom_foot_neighbors and placed_shingle_location not in top_foot_neighbors:
                        # move the top foot to the placed shingle
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                        self.claim_pos(real_roof, bottom_foot_shingle_neighbors[0]["pos"])
                        claimed_new_postion = True
                        self.old_top_foot = self.top_foot_position
                    # for additional checks, make sure that both shingle lists have at least one entry
                    elif len(bottom_foot_shingle_neighbors) > 0 and len(top_foot_shingle_neighbors) > 0:
                        # if bottom_foot is farther away, move bottom_foot
                        if Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) > Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target) and top_foot_shingle_neighbors[0]["pos"] != placed_shingle_location:
                            self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof,top_foot_shingle_neighbors[0]["pos"])
                           #rospy.loginfo(f"inchworm {self.id} wants to move it's bottom foot")

                            claimed_new_postion = True
                            self.old_bottom_foot = self.bottom_foot_position
                        # if top_foot is farther away, move top_foot
                        elif Inchworm.dist(bottom_foot_shingle_neighbors[0]["pos"], self.target) < Inchworm.dist(top_foot_shingle_neighbors[0]["pos"], self.target) and bottom_foot_shingle_neighbors[0]["pos"] != placed_shingle_location:
                            self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_uninstalled_foot_positions, real_roof)
                            self.claim_pos(real_roof, bottom_foot_shingle_neighbors[0]["pos"])
                           #rospy.loginfo(f"inchworm {self.id} wants to move it's top foot")
                            claimed_new_postion = True
                            self.old_top_foot = self.top_foot_position
                        else:
                            self.target = [random.randint(0, self.roof_width), random.randint(0, len(self.roof[0]))]

                            if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                                self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
                            else:
                                self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
                            # if the inchworm is not installing a shingle, initate movement of the inchworm to the target
                            
                            # checks to make sure that ee_shingle_neighbors has a valid movement option
                            if (len(self.ee_shingle_neighbors) > 0 and 
                                self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED):
                                # rospy.loginfo(f"inchworm {self.id} is claiming {self.ee_shingle_neighbors[0]['pos']} and initate move")
                                self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                                self.state_counts["move_to_target"] += 1
                                self.robot_state = RobotState.MOVE_TO_TARGET
                                self.stuck_count = 0
                            # elif random.random() > 0.25:
                            #     self.make_state_explore(real_roof)


                    else:
                        pass
                        #rospy.logwarn(
                            #f"inchworm {self.id} could not figure out which end effector to use to move the shingle at {placed_shingle_location}")

                    self.move_shingle_step = 1
                    self.probe_step = 0
                    self.original_bottom_foot_pos = self.bottom_foot_position
                    self.original_top_foot_pos = self.top_foot_position
                    # rospy.loginfo(f"placed shingle occ grid {real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord])}")
                    # rospy.loginfo(f"robot {self.id} clamed new posiiton {claimed_new_postion}")
                    # try and claim the placed shingle if the inchworm is trying to move a shingle
                    if claimed_new_postion and real_roof.get_occ_position([placed_shingle.x_coord, placed_shingle.y_coord]) == 0:
                        self.claim_pos(real_roof,[placed_shingle.x_coord, placed_shingle.y_coord])
                        self.robot_state = RobotState.MOVE_SHINGLE
                        self.state_counts["move_shingle"] += 1
                        # rospy.loginfo(f"inchworm {self.id} moving shingle")
                    # else:
                    #     # otherwise try and move away, this will sometimes throw an exception due to unclaiming, if it does the inchworm should not move
                    #     try:
                    #         self.unclaim_pos(real_roof, [placed_shingle.x_coord, placed_shingle.y_coord])
                    #     except Exception as e:
                    #         #rospy.logwarn(f"inchworm {self.id} encountered an exception {e}")
                    #     self.make_state_explore(real_roof)

            else:
                # if the inchworm is not next to the shingle depot, move toward it, otherwise pick up a new shingle
                if not self.next_to_shingle_depot(real_roof.get_shingle_depot_location(False)):
                    # rospy.loginfo(f"inchworm {self.id} moving towards depot")
                    self.make_state_move_to_depot(real_roof)
                else:
                    # rospy.loginfo(f"inchworm {self.id} is picking up a new shingle from the depot")
                    self.robot_state = RobotState.PICKUP_SHINGLE_FROM_DEPOT
                    self.state_counts["pickup_from_depot"] += 1

        return real_roof

    def run_action(self, real_roof):
        # this is where all movement will happen

        if self.robot_state == RobotState.PICKUP_SHINGLE_FROM_DEPOT:
            # place holder for now, shingle depot just spawns a new shingle in the persumed location

            rospy.sleep(Inchworm.DELAY)
            # rospy.loginfo(f"shingle depot at {real_roof.get_shingle_depot_location(False)}")
            # check if the shingle depot has placed a shingle in the new spot
            if real_roof.spawn_shingle(self.id): # TODO: depot currently spawns the shingle in the new location, this will need to change
                self.set_shingle_state(0, self.shingle_depot_pos[0] + 1, ShingleStatus.PLACED)
                self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                self.robot_state = RobotState.MAKE_DECISION
            elif self.next_to_placed_shingle(self.bottom_foot_position) or self.next_to_placed_shingle(self.top_foot_position):
               #rospy.loginfo(f"inchworm {self.id} failed to spawn shingle, but is next to a spawned shingle")
                self.robot_state = RobotState.MAKE_DECISION
            else:
                if random.random() < PROB_OF_EXPLORE_IF_CANT_MOVE:
                    pass
                self.make_state_explore(real_roof)

            
        elif self.robot_state == RobotState.MOVE_TO_TARGET:
            # look the foot shingle neighbors to determain which end effector is being moved to target
                    
            
            status = self.probe(real_roof, self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
            if status[0] == 0:
                if status[1] and self.get_shingle_state(self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][0], self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][1]) == ShingleStatus.INSTALLED:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                        # check if foot is not at the target, if so move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                            self.old_bottom_foot = self.bottom_foot_position
                            self.move_bottom_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # otherwise plant the foot and unclaim the old position
                            if self.bottom_foot_position != self.old_bottom_foot:
                                # self.unclaim_pos(real_roof, self.old_bottom_foot)
                                self.old_bottom_foot = self.bottom_foot_position
                            self.bottom_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                    else:
                        # check if foot is not at the target, if so move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                            self.old_top_foot = self.top_foot_position
                            self.move_top_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # otherwise plant the foot and unclaim the old position
                            if self.top_foot_position != self.old_top_foot:
                                # self.unclaim_pos(real_roof, self.old_top_foot)
                                self.old_top_foot = self.top_foot_position
                            self.top_foot_status = EEStatus.PLANTED
                            self.robot_state = RobotState.MAKE_DECISION
                else:
                    self.set_shingle_state(self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][0], self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][1], ShingleStatus.UNINSTALLED)

                    if self.old_bottom_foot != None and self.old_bottom_foot != self.bottom_foot_position:
                        self.move_bottom_foot(self.old_bottom_foot)
                        
                        if self.bottom_foot_position == self.old_bottom_foot:
                            self.bottom_foot_status = EEStatus.PLANTED
                            self.set_shingle_state(self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][0], self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][1], ShingleStatus.UNINSTALLED)
                            self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                            self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                            self.robot_state = RobotState.MAKE_DECISION
                    if self.old_top_foot != None and self.old_top_foot != self.top_foot_position:
                        self.move_top_foot(self.old_top_foot)
                        
                        if self.top_foot_position == self.old_top_foot:
                            self.top_foot_status = EEStatus.PLANTED
                            self.set_shingle_state(self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][0], self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"][1], ShingleStatus.UNINSTALLED)
                            self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                            self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                            self.robot_state = RobotState.MAKE_DECISION

        elif self.robot_state == RobotState.INSTALL_SHINGLE:
            # rospy.loginfo(f"inchworm {self.id} is in install state {self.installing_status}")
            if self.placed_shingle_is_valid([self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]):
                # double check that the placed shingle is a valid install 
                # rospy.loginfo(
                #     f"inchworm {self.id} is installing a shingle at {self.target}")
                # installation is a two step process
                # small state machine to allow for multi step install
                '''
                create pre step where we move to the target, and check if there is a shingle there
                    - set original foot positions
                    - probe the potental point
                    - if 
                '''
               #rospy.loginfo(f"inchworm {self.id} has a shingle in a valid install location")

                if self.installing_status == 1:
                    self.original_bottom_foot_pos = self.bottom_foot_position
                    self.original_top_foot_pos = self.top_foot_position

                    # figure out which foot needs to install the shingle, this needs to be done once per install
                    if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                        self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_foot_positions, real_roof)
                        if len(self.ee_shingle_neighbors) >= 1:
                            self.installing_status = 2
                        else:
                            self.installing_status = 0
                            self.robot_state = RobotState.MAKE_DECISION
                    else:
                        self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_foot_positions, real_roof)
                        if len(self.ee_shingle_neighbors) >= 1:
                            self.installing_status = 2
                        else:
                            self.installing_status = 0
                            self.robot_state = RobotState.MAKE_DECISION
                elif self.installing_status == 2:
                    
                    status = self.probe(real_roof, self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    if status[0] == 0:
                        if status[1]:
                            #rospy.logwarn(f"inchworm {self.id} found an shingle install location at {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.installing_status = 3
                        else:
                            #rospy.logwarn(f"inchworm {self.id} did not find an shingle at install loc {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.UNINSTALLED)
                            if self.bottom_foot_position[0] == self.install_shingle_target.x_coord and self.bottom_foot_position[1] == self.install_shingle_target.y_coord:
                                self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                            else:
                                self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                            self.installing_status = 6
                elif self.installing_status == 3:
                    status = self.probe(real_roof, [self.install_shingle_target.x_coord, self.install_shingle_target.y_coord])
                    if status[0] == 0:
                        if status[1]:
                            #rospy.logwarn(f"inchworm {self.id} found an shingle install location at {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.installing_status = 4
                        else:
                            #rospy.logwarn(f"inchworm {self.id} did not find an shingle at install loc {[self.install_shingle_target.x_coord, self.install_shingle_target.y_coord]}")
                            self.set_shingle_state(self.install_shingle_target.x_coord, self.install_shingle_target.y_coord, ShingleStatus.UNINSTALLED)
                            if self.bottom_foot_position[0] == self.install_shingle_target.x_coord and self.bottom_foot_position[1] == self.install_shingle_target.y_coord:
                                self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                            else:
                                self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                            self.installing_status = 6

                    # self.installing_status = 3
                # install the shingle and place the foot on the new shingle
                elif self.installing_status == 4: 
                    # determains which foot we are controling 
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                            self.old_bottom_foot = self.bottom_foot_position
                            self.move_bottom_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            self.install_shingle(EE.BOTTOM_FOOT, self.install_shingle_target, real_roof)               
                            self.installing_status = 5

                    else:
                        # insure that the foot is in the correct location, if not we move the foot
                        if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                            self.old_top_foot = self.top_foot_position
                            self.move_top_foot(
                                self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                        else:
                            # install the shingle and place the foot on the newly installed shingle
                            self.install_shingle(EE.TOP_FOOT, self.install_shingle_target, real_roof)
                            self.installing_status = 5
                elif self.installing_status == 5:
                    if self.original_bottom_foot_pos != self.bottom_foot_position:
                        self.move_bottom_foot(self.original_bottom_foot_pos)
                        self.bottom_foot_status = EEStatus.PLANTED
                    elif self.original_top_foot_pos != self.top_foot_position:
                        self.move_top_foot(self.original_top_foot_pos)
                        self.top_foot_status = EEStatus.PLANTED
                    # if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    #     self.unclaim_pos(real_roof, self.old_bottom_foot)
                    # else:
                    #     self.unclaim_pos(real_roof, self.old_top_foot)
                    # self.unclaim_pos(real_roof, [self.install_shingle_target.x_coord, self.install_shingle_target.y_coord])
                    else:
                        self.robot_state = RobotState.MAKE_DECISION
                        self.installing_status = 0
                elif self.installing_status == 6:
                    if self.original_bottom_foot_pos != self.bottom_foot_position:
                        self.move_bottom_foot(self.original_bottom_foot_pos)
                        self.bottom_foot_status = EEStatus.PLANTED
                    elif self.original_top_foot_pos != self.top_foot_position:
                        self.move_top_foot(self.original_top_foot_pos)
                        self.top_foot_status = EEStatus.PLANTED
                    else:
                    # try:
                    #     self.unclaim_pos(real_roof, self.old_top_foot)
                    # except Exception:
                    #     pass
                    # try:
                    #     self.unclaim_pos(real_roof, self.old_bottom_foot)
                    # except Exception:
                    #     pass
                        self.robot_state = RobotState.MAKE_DECISION
                        self.installing_status = 0



        elif self.robot_state == RobotState.PATROL_FRONTIER:
            # TODO: This is where we could implement moving along the frontier, just moving about
            self.robot_state = RobotState.MAKE_DECISION
        elif self.robot_state == RobotState.MOVE_SHINGLE:
            # rospy.logwarn(f"inchworm {self.id} is in move shingle step {self.move_shingle_step}")
            # moving shingles is a multi step process so it requires a state machine in order to be non-blocking
            if self.move_shingle_step == 1:
                    
                status = self.probe(real_roof, [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                if status[0] == 0:
                    if status[1]:
                       #rospy.loginfo(f"inchworm {self.id} found an shingle at {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_shingle_step = 2
                    else:
                       #rospy.loginfo(f"inchworm {self.id} did not find a placed shingle at {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.set_shingle_state(self.shingle_to_move.x_coord, self.shingle_to_move.y_coord, ShingleStatus.UNINSTALLED)
                        if self.bottom_foot_position[0] == self.shingle_to_move.x_coord and self.bottom_foot_position[1] == self.shingle_to_move.y_coord:
                            self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        else:
                            self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.move_shingle_step = 6
                    
        
            elif self.move_shingle_step == 2:
                
                status = self.probe(real_roof, self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                if status[0] == 0:
                    if not status[1]:
                       #rospy.loginfo(f"inchworm {self.id} did not find a shingle at {self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']}")
                        self.old_shingle_pos = [
                            self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]
                        self.move_shingle_step = 3
                    else:
                       #rospy.loginfo(f"inchworm {self.id} found an shingle at {self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']}")
                        # create new shingle in this position
                        shingle_target_postion = self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]['pos']
                        self.read_shingle_at(real_roof, shingle_target_postion)
                        if self.bottom_foot_position[0] == shingle_target_postion[0] and self.bottom_foot_position[1] == shingle_target_postion[1]:
                            self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        else:
                            base_shingle = real_roof.get_shingle(self.bottom_foot_position[0], self.bottom_foot_position[1])
                            self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.move_shingle_step = 6
                    
            elif self.move_shingle_step == 3:
                

                # check to see which foot you need to move
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.bottom_foot_position and self.shingle_to_move.x_coord != -1:
                        # rospy.loginfo(
                        #     f"inchworm {self.id} moving bottom_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_bottom_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                    else:
                        if self.shingle_to_move.x_coord != -1:
                            self.pickup_shingle(EE.BOTTOM_FOOT, self.shingle_to_move, real_roof)
                        else:
                            self.move_shingle_step = 4
                else:
                    # move the foot to the new position and pick up a shingle
                    if [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord] != self.top_foot_position and self.shingle_to_move.x_coord != -1:
                        # rospy.loginfo(
                        #     f"inchworm {self.id} moving top_foot to {[self.shingle_to_move.x_coord, self.shingle_to_move.y_coord]}")
                        self.move_top_foot(
                            [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                        
                    else:
                        if self.shingle_to_move.x_coord != -1:
                            self.pickup_shingle(EE.TOP_FOOT, self.shingle_to_move, real_roof)
                        else:
                            self.move_shingle_step = 4
                        
                        

            elif self.move_shingle_step == 4:
                # move the foot to the new location
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.bottom_foot_position:
                        self.move_bottom_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        if self.shingle_to_move.x_coord == -1:
                            self.place_shingle(EE.BOTTOM_FOOT, self.shingle_to_move, real_roof)
                        else:
                            self.move_shingle_step = 5
                # moves the foot to the new location
                else:
                    if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"] != self.top_foot_position:
                        self.move_top_foot(
                            self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["pos"])
                    else:
                        # place the shingle
                        if self.shingle_to_move.x_coord == -1:
                            self.place_shingle(EE.TOP_FOOT, self.shingle_to_move, real_roof)
                        else:
                            self.move_shingle_step = 5

            else:
                # moves the foot to it's old postion
                if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.original_bottom_foot_pos)
                    if self.original_bottom_foot_pos == self.bottom_foot_position:
                        self.bottom_foot_status = EEStatus.PLANTED
                        self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        # rospy.loginfo(f"shingles claimed after unclaiming {self.claimed_pos}")
                        self.robot_state = RobotState.MAKE_DECISION
                        self.move_shingle_step = 0
                        self.probe_step = 0

                else:
                    self.move_top_foot(self.original_top_foot_pos)
                    if self.original_top_foot_pos == self.top_foot_position:
                        self.top_foot_status = EEStatus.PLANTED
                        self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        # rospy.loginfo(f"shingles claimed after unclaiming {self.claimed_pos}")
                        self.robot_state = RobotState.MAKE_DECISION
                        self.move_shingle_step = 0
                        self.probe_step = 0
                # un claim the positions that it was using
                # try:
                #     self.unclaim_pos(real_roof, [self.shingle_to_move.x_coord, self.shingle_to_move.y_coord])
                # except Exception as e:
                #     #rospy.logwarn(e)
                #     #rospy.logwarn(self.move_shingle_step)
                # try:
                #     self.unclaim_pos(real_roof,self.old_shingle_pos)
                # except Exception as e:
                #     #rospy.logwarn(e)
                #     #rospy.logwarn(self.move_shingle_step)
                
        elif self.robot_state == RobotState.EXPLORE:
            if self.explore_state == 1:
                status = self.probe(real_roof, self.ee_shingle_neighbors[0]["pos"])
                if status[0] == 0:
                    if status[1]:
                       #rospy.loginfo(f"inchworm {self.id} found an shingle at {self.ee_shingle_neighbors[0]['pos']}")
                        self.read_shingle_at(real_roof, self.ee_shingle_neighbors[0]["pos"])
                        # if self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.UNINSTALLED:
                        #     self.set_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1], ShingleStatus.PLACED)
                        self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.explore_state = 3
                    else:
                       #rospy.loginfo(f"inchworm {self.id} did not find an shingle at {self.ee_shingle_neighbors[0]['pos']}")
                        self.set_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1], ShingleStatus.UNINSTALLED)

                        self.explore_state = 2
            elif self.explore_state == 2:
                if self.ee_shingle_neighbors[0]['foot'] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.old_bottom_foot)
                    if self.bottom_foot_position == self.old_bottom_foot:
                        self.explore_state = 4
                        self.bottom_foot_status = EEStatus.PLANTED
                else:
                    self.move_top_foot(self.old_top_foot)
                    if self.top_foot_position == self.old_top_foot:
                        self.explore_state = 4
                        self.top_foot_status = EEStatus.PLANTED
                # try:
                #         self.unclaim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                # except Exception:
                #     pass
                

                
            elif self.explore_state == 3:
                if self.ee_shingle_neighbors[0]['foot'] == EE.BOTTOM_FOOT:
                    self.move_bottom_foot(self.old_bottom_foot)
                    if self.bottom_foot_position == self.old_bottom_foot:
                        self.bottom_foot_status = EEStatus.PLANTED
                        self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                        self.explore_state = 4
                else:
                    self.move_top_foot(self.old_top_foot)
                    if self.top_foot_position == self.old_top_foot:
                        self.top_foot_status = EEStatus.PLANTED
                        self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                        self.explore_state = 4
                
            elif self.explore_state == 4:
                if Inchworm.dist(self.bottom_foot_position, self.target) > Inchworm.dist(self.top_foot_position, self.target):
                    self.decide_on_movement_to_shingle(EE.BOTTOM_FOOT, self.valid_installed_foot_positions, real_roof)
                else:
                    self.decide_on_movement_to_shingle(EE.TOP_FOOT, self.valid_installed_foot_positions, real_roof)
                # if the inchworm is not installing a shingle, initate movement of the inchworm to the target
                
                # checks to make sure that ee_shingle_neighbors has a valid movement option
                if (len(self.ee_shingle_neighbors) > 0 and 
                    self.get_shingle_state(self.ee_shingle_neighbors[0]["pos"][0], self.ee_shingle_neighbors[0]["pos"][1]) == ShingleStatus.INSTALLED):
                    # rospy.loginfo(f"inchworm {self.id} is claiming {self.ee_shingle_neighbors[0]['pos']} and initate move")
                    self.claim_pos(real_roof, self.ee_shingle_neighbors[0]["pos"])
                    self.robot_state = RobotState.MOVE_TO_TARGET
                    self.stuck_count = 0
                else:
                    self.robot_state = RobotState.MAKE_DECISION
                self.explore_state = 0
            else:
                self.update_shingle_with_current_roof(real_roof, self.top_foot_position)
                self.update_shingle_with_current_roof(real_roof, self.bottom_foot_position)
                self.robot_state = RobotState.MAKE_DECISION
                self.explore_state = 0
                pass

            pass

        

        return real_roof

    # returns true if there is a shingle at that location
    def probe(self, real_roof, shingle_probe_pos):
        '''
            - move to shingle to probe if you are not already there
            - gets the status of the shingle from the roof
        '''

        # rospy.loginfo(f"inchworm {self.id} probe shingle step of {self.probe_step}")
        if self.probe_step == 0:
            self.probe_status = False
            self.old_shingle_pos = shingle_probe_pos
            if self.ee_shingle_neighbors[self.foot_shingle_neighbor_to_move_to]["foot"] == EE.BOTTOM_FOOT:
                if shingle_probe_pos != self.bottom_foot_position:
                    # rospy.loginfo(f"inchworm {self.id} moving bottom foot")
                    self.old_bottom_foot = self.bottom_foot_position
                    self.move_bottom_foot(shingle_probe_pos)
                else:
                    self.probe_step = 1
            else:
                # rospy.loginfo(f"inchworm {self.id} {shingle_probe_pos}, {self.top_foot_position}")
                if shingle_probe_pos != self.top_foot_position:
                    # rospy.loginfo(f"inchworm {self.id} moving top foot")
                    self.old_top_foot = self.top_foot_position
                    self.move_top_foot(shingle_probe_pos)
                else:
                    self.probe_step = 1
        elif self.probe_step == 1:
            self.probe_status = (real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]) is not None and (real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]).shingle_status == ShingleStatus.INSTALLED or
                real_roof.get_shingle(shingle_probe_pos[0], shingle_probe_pos[1]).shingle_status == ShingleStatus.PLACED))
            self.probe_step = 2
        else:
            
            self.probe_step = 0
        if self.probe_step == 2:
            return (0, self.probe_status)
        else:
            return (-1, False)
    
    def check_shingle_state(self, real_roof, pos):
        self.read_shingle_at(real_roof, pos)
        return self.get_shingle_state(pos[0], pos[1])


    def decide_on_movement_to_shingle(self, ee_to_move, neighbor_funtion, real_roof):
        '''decides on how to move a given foot to a shingle
            right now this uses greedy movement and in the future we could have it look a few shingles ahead to determain a better path
        '''
        if ee_to_move == EE.BOTTOM_FOOT:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_shingle_pos_neighbors(self.top_foot_position), EE.BOTTOM_FOOT, real_roof)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_bottom_foot = self.bottom_foot_position
        else:
            self.ee_shingle_neighbors = neighbor_funtion(
                self.get_shingle_pos_neighbors(self.bottom_foot_position), EE.TOP_FOOT, real_roof)
            self.ee_shingle_neighbors.sort(
                key=lambda x: Inchworm.dist(x["pos"], self.target))
            self.old_top_foot = self.top_foot_position
        if len(self.ee_shingle_neighbors) < 1:
           rospy.loginfo(f"inchworm {self.id} can't move????")
        self.foot_shingle_neighbor_to_move_to = 0



    def to_message(self):
        msg = InchwormMsg()
        msg.id = self.id
        msg.bottom_foot_pos = self.bottom_foot_position
        msg.top_foot_pos = self.top_foot_position
        msg.bottom_foot_status = self.bottom_foot_status.value
        msg.top_foot_status = self.top_foot_status.value
        msg.bottom_foot_shingle_stat = self.bottom_foot_shingle_stat.value
        msg.top_foot_shingle_stat = self.top_foot_shingle_stat.value
        msg.behavior = self.behavior.value
        if self.bottom_foot_status == EEStatus.PLANTED:
            msg.bottom_foot_valid_neighbors = sum(
                self.get_shingle_pos_neighbors(self.bottom_foot_position), [])
        else:
            msg.bottom_foot_valid_neighbors = []

        if self.top_foot_status == EEStatus.PLANTED:
            msg.top_foot_valid_neighbors = sum(
                self.get_shingle_pos_neighbors(self.top_foot_position), [])
        else:
            msg.top_foot_valid_neighbors = []
        msg_roof = []
        for row in self.roof:
            for shingle in row:
                msg_roof.append(shingle.value)
        msg.roof = msg_roof
        # msg.roof = self.roof
        msg.roof_width = self.roof_width
        msg.shingle_depot_pos = self.shingle_depot_pos
        if self.target is not None:
            msg.target = self.target
        msg.header.stamp = rospy.Time.now()
        msg.state = self.robot_state.value
        msg.roof_conf = self.roof_confidence
        msg.claimed = [item for sublist in self.claimed_pos for item in sublist]
        return msg

    def tile_path(self, start, goal):
        frontier_path = self.calc_frontier(self.roof)
        # rospy.loginfo(f"frontier is {frontier_path}")
        path_to_goal = [[start[0], start[1], Inchworm.dist(start, goal)]]
        # rospy.sleep(10)

        while (path_to_goal[-1][0] is not goal[0] or path_to_goal[-1][1] is not goal[1]) and (
            self.get_shingle_state(goal[0], goal[1]) is not ShingleStatus.INSTALLED or self.get_shingle_state(goal[0], goal[1]) is not ShingleStatus.PLACED): 
            neighbors = []
            for point in frontier_path:
                if self.is_neighbor(path_to_goal[-1][0:2], point):
                    neighbors.append([point[0], point[1], Inchworm.dist(point, goal)])

            # rospy.loginfo(f"looking at neighbors{neighbors}")

            next_best_tile = path_to_goal[-1]
            # rospy.loginfo(f"starting best tile is {next_best_tile}")

            for adjacent in neighbors:
                # rospy.loginfo(f"adjacent is {adjacent}")

                if adjacent[2]  < next_best_tile[2]:
                    next_best_tile = adjacent
                    
            # rospy.loginfo(f"best tile is {next_best_tile}")
            if not (next_best_tile[0] is path_to_goal[-1][0] and next_best_tile[1] is path_to_goal[-1][1]):
                path_to_goal.append(next_best_tile)
            else:
                #rospy.logwarn(f"inchworm {self.id} has a goal of {goal} from path planning with path {path_to_goal}")
                # for i in reversed(range(len(self.roof[0]))):
                #     row = []
                #     for j in range(len(self.roof)):
                #         row.append(self.roof[j][i].value)
                #     #rospy.logwarn(f"inchworm {self.id} row {i} has values {row}")
                #rospy.logwarn(f"inchworm {self.id} PATH STUCK IN A DEAD END, CHECK NEIGHBORS")
                # rospy.loginfo(f"start is {start}, goal is {goal}, path is {path_to_goal}")
                oh_no = []
                oh_no.append(start)
                oh_no.append(goal)
                return oh_no


        shingle_coord_path = []

        for step in path_to_goal:
            shingle_coord_path.append([step[0], step[1]]);
        
        # rospy.loginfo(f"path to get to the goal along the frontier is {shingle_coord_path}")

        return shingle_coord_path

    def calc_frontier(self, robot_map):
        frontier_shingles = []
        
        for width in range(len(robot_map)):
            for height in range(len(robot_map[width])):
                if self.next_to_installed_shingle([width,height], robot_map) and robot_map[width][height] is not ShingleStatus.INSTALLED:
                    # rospy.loginfo(f"the shingle being appended to the frontier ({[width,height]}) list is status {robot_map[width][height]}")
                    frontier_shingles.append([width, height])
       #rospy.loginfo(f"inchworm {self.id} has a frontier of {frontier_shingles}")
        return frontier_shingles

    def next_to_installed_shingle(self, pos, shingles):
        neighbors = self.get_neighbors_of_a_shingle(pos)
        for n in neighbors:
            read_shingle = shingles[n[0]][n[1]]
            if read_shingle is not None:
                if read_shingle == ShingleStatus.INSTALLED:
                    return True
        return False

    def get_neighbors_of_a_shingle(self, pos):
        neighbor_pos = []
        if pos[1] % 2 == 0:  # even row lookup
            for n in Inchworm.EVEN_ROW_N_LOOKUP:
                new_neighbor_pos = [pos[0] + n[0], pos[1] + n[1]]
                if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof[0]) and new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                    neighbor_pos.append(new_neighbor_pos)
        else:
            for n in Inchworm.ODD_ROW_N_LOOKUP:
                new_neighbor_pos = [pos[0] + n[0], pos[1] + n[1]]
                if new_neighbor_pos[0] < self.roof_width and new_neighbor_pos[1] < len(self.roof[0]) and new_neighbor_pos[0] > -1 and new_neighbor_pos[1] > -1:
                    neighbor_pos.append(new_neighbor_pos)
        return neighbor_pos

    def is_neighbor(self, origin, pos):
        neighbors = self.get_neighbors_of_a_shingle(origin)
        for shingle in neighbors:
            if shingle[0] == pos[0] and shingle[1] == pos[1]:
                return True

        return False

    def get_installed_neighbors(self, pos):
        neighbors = self.get_neighbors_of_a_shingle(pos)
        installed_neighbors = []
        for n in neighbors:
            read_shingle = self.roof[n[0]][n[1]]
            if read_shingle is not None:
                if read_shingle == ShingleStatus.INSTALLED:
                    installed_neighbors.append(n)
        if len(installed_neighbors) == 0:
            #rospy.logwarn(f"Shingle at {pos} has no installed neighbors")
            pass

        return installed_neighbors

    def get_intersection(list1, list2):
        intersection = []
        for x in list1:
            for y in list2:
                if x[0] == y[0] and x[1] == y[1]:
                    intersection.append(x)

        return intersection

    def inchworm_spot_to_move_shingle(self, shingle, goal):
        neighbors_of_shingle = self.get_installed_neighbors(shingle)
        neighbors_of_goal = self.get_installed_neighbors(goal)
        spot = Inchworm.get_intersection(neighbors_of_shingle, neighbors_of_goal)
        if len(spot) == 0:
            #rospy.logwarn(f"INVALID path, no spot for the robot to move the shingle {shingle} to the next point {goal}")
            return neighbors_of_shingle[0]
        return spot[0]

    '''
    TODO:
        - get rid of all magic functions
            - this will require fixing reading shingles & storage of local roof copies
            - will also require a working probe roof subtask && making sure that when a inchworm installs a shingle it rewrites all neighbors
        - read and write info to roof
        - implement the hex coords
        - create algo that gets the current frontier - involves getting all valid shingle locations based on current roof
        - possible fix for the issue when running 3 worms and 2 moves weirdly
            - actual path planning towards target
            - avoid problem by having an inchworm move first to the frontier and then to the shingle depot
        - create some viz for target

        - path planning so that the inchworm does not move in a greedy fasion, this should treat all other inchworms as obsticals but will only need to run one tick at a time
        - make a new function for choosing the target tile based on the list of valid shingle targets - this is how we can create different patterns


        sudo apt-get install python-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev libsmpeg-dev python-numpy subversion libportmidi-dev ffmpeg libswscale-dev libavformat-dev libavcodec-dev libfreetype6-dev
        pip install pygame==1.9.6


        - possible optimizations
            - plant a foot after installing || finding an installed shingle
            - time stamp data about the shingles, this will allow us to keep up to date info
            - record the inchworms target in the shingle, this will allow inchworm to do more intlegent comms
    '''

if __name__ == "__main__":
    iw = Inchworm(width=10, height=10)
    print(iw.create_physics_order(5, 5))