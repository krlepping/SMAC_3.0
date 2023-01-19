#!/usr/bin/env python3

import rospy, rospkg, math, shingle, sys, std_msgs, actionlib


from inchworm_algo.msg import ShingleMsg
from inchworm_algo.srv import *
from shingle import Shingle



class ShingleDepot():
    depot_position = 0
    depot_on_opposite_side = False
    roof = None

    def __init__(self, roof, is_on_opposite_side) -> None:
        self.roof = roof
        self.depot_position = 0
        self.depot_on_opposite_side = is_on_opposite_side
    

    def get_location(self):
        return self.depot_position
    
    def move_shingle_depot_up(self):
        self.depot_position += 1
        return self.depot_position
    
    def create_shingle(self, is_half_shingle, shingle_count): 
        # check if inchworm is next to shingle depot
        shingle_count += 1
        return Shingle(shingle_count, is_half_shingle), shingle_count





