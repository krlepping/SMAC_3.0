#!/usr/bin/env python3
import rospy, actionlib
from shingle_depot import ShingleDepot
from shingle import Shingle, ShingleStatus, NeighborIndex
from inchworm_control.msg import InchwormAction, InchwormGoal
from actionlib_msgs.msg import GoalStatus


# all x and y are in array coords currently



### NOTICE: COLLISION AVOIDENCE IS CURRENTLY CENTILIZED

from inchworm_algo.msg import ShingleMsg, RoofState, InchwormMsg
import rospy

class Roof():
    # these arrays are used to lookup the correct NeighborIndex based on the difference in array coords
    # rows are 0 indexed
    # 0 starts on the left hand side of the roof
    # half shingles are on the right side of even rows and on the left side of odd rows
    # see even- r on https://www.redblobgames.com/grids/hexagons/
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    width = 0
    height = 0
    shingle_array = [[]]
    shingle_depots = []
    inchworms = []
    inchworm_occ = []
    shingle_count = -1

    def __init__(self, width, height, dual_side_depots, physics = False):
        self.shingle_array = []
        for i in range(height):
            self.shingle_array.append([None] * width)
            self.inchworm_occ.append([0] * width)
        self.width = width
        self.height = height
        self.spawn_first_row()
        self.spawn_depots(dual_side_depots)
        self.inchworms = []
        self.using_physics = physics
        


    def place_shingle(self, shingle, coord):
        shingle = shingle.place_shingle(coord[0], coord[1])
        self.shingle_array[coord[1]][coord[0]] = shingle
        return shingle


    def pickup_shingle(self, coord):
        if coord[0] >= 0 and coord[1] >= 0 and self.shingle_array[coord[1]][coord[0]] is not None:
            shingle = self.shingle_array[coord[1]][coord[0]].pickup_shingle()
            self.shingle_array[coord[1]][coord[0]] = None
        else:
            #rospy.logwarn(f"Shingle at location {[coord[0], coord[1]]} was not marked as being placed in the roof")
            shingle = self.shingle_array[coord[1]][coord[0]]
        return shingle

    def install_shingle(self, shingle):
        shingle = shingle.install_shingle(shingle.x_coord, shingle.y_coord)
        self.shingle_array[shingle.y_coord][shingle.x_coord] = shingle
        return shingle


    def get_shingle(self, x, y):
        return self.shingle_array[y][x]
    
    def set_shingle(self, x, y, shingle):
        
        self.shingle_array[y][x] = shingle
    
    def get_shingle_n_index(self, t_x, t_y, n_x, n_y):
        delta_x = t_x - n_x
        delta_y = t_y - n_y
        if t_x % 2 == 0:
            return self.EVEN_ROW_N_LOOKUP.index((delta_x, delta_y))
        else:
            return self.ODD_ROW_N_LOOKUP.index((delta_x, delta_y))


    def spawn_first_row(self):
        #rospy.loginfo("spawining first row of shingles")
        for i in range(self.width):
            is_half_shingle = False
            if i + 1 == self.width:
                is_half_shingle == True
            new_shingle = Shingle(i, is_half_shingle)
            new_shingle = new_shingle.install_shingle(i, 0)
            new_shingle.update_neighbor(0, ShingleStatus.INSTALLED)
            new_shingle.update_neighbor(3, ShingleStatus.INSTALLED)
            self.shingle_array[0][i] = new_shingle
        self.shingle_count = self.width
        for shingleList in self.shingle_array:
            for shingle in shingleList:
                if shingle is not None:
                    print(shingle.shingle_status)
        # TODO: FOLLOWING IS JUST FOR A VIS
        # new_shingle = Shingle(i, is_half_shingle)
        # new_shingle.place_shingle(1, 1)
        # self.shingle_array[1][1] = new_shingle
        # self.shingle_count += 1

    def claim_position(self, coord):
        self.inchworm_occ[coord[1]][coord[0]] = 1

    def unclaim_position(self, coord):
        self.inchworm_occ[coord[1]][coord[0]] = 0

    def get_occ_position(self, coord):
        return self.inchworm_occ[coord[1]][coord[0]]
        
    def increment_shingle_count(self):
        self.shingle_count += 1
        return self.shingle_count


    # def update_shingle_neighbor(self, target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y, n_id, n_status):
    #     # figure out what the neighbor is based on x, y 
    #     target_shingle = self.shingle_array[target_shingle_y][target_shingle_x]
    #     n_index = self.get_shingle_n_index(target_shingle_x, target_shingle_y, shingle_n_x, shingle_n_y)
    #     target_shingle.update_neighbor(n_id, n_index, n_status)
        

    def spawn_depots(self, spawn_opposite_side):
        self.shingle_depots.append(ShingleDepot(self, False))
        if spawn_opposite_side:
            self.shingle_depots.append(ShingleDepot(self, True))
        pass

    def move_shingle_depot(self, opposite_side):
        if opposite_side:
            self.shingle_depots[1].move_shingle_depot_up()
        else:
            self.shingle_depots[0].move_shingle_depot_up()


    def check_if_can_spawn_shingle(self):
        if self.get_shingle(0, self.shingle_depots[0].get_location() + 1) != None:
            #rospy.loginfo(self.get_shingle(0, self.shingle_depots[0].get_location() + 1).shingle_status)
            if (self.get_shingle(0, self.shingle_depots[0].get_location() + 1).shingle_status == ShingleStatus.INSTALLED):
                #rospy.loginfo("moving depot up")
                self.move_shingle_depot(False)
            return False
        else:
            return True

    def spawn_shingle(self, inchworm_id):
        if self.check_if_can_spawn_shingle():
            self.shingle_array[self.shingle_depots[0].get_location() + 1][0], self.shingle_count = self.shingle_depots[0].create_shingle(False, self.shingle_count)
            self.shingle_array[self.shingle_depots[0].get_location() + 1][0].place_shingle(0, self.shingle_depots[0].get_location() + 1)
            if self.using_physics:
                self.action_client = actionlib.SimpleActionClient(f"/inchworm_action_{inchworm_id}", InchwormAction)
                self.action_client.wait_for_server()
                goal = InchwormGoal()
                goal.action_type = 3
                
                goal.coord_x = 0
                goal.coord_y = self.shingle_depots[0].get_location() + 1
                
                goal.end_effector = 0
                self.action_client.send_goal(goal)
            return True
        else:
            return False

    def get_shingle_depot_location(self, opposite_side):
        if opposite_side:
            return self.shingle_depots[1].get_location()
        else:
            return self.shingle_depots[0].get_location()


    def to_message(self):
        # TODO: include inchworms & shingle depot in message
        roof_state = RoofState()
        roof_state.width = self.width
        roof_state.height = self.height
        shingle_array_temp = []
        # this converts the shingle array in the roof to an occupancy grid
        for i in range(self.height):
            shingle_array_temp.extend(self.shingle_array[i])
        shingle_array_msg = []
        # rospy.loginfo(self.shingle_array)

        for shingle in shingle_array_temp:
            if shingle is None:
                if self.neighbor_is_placed(len(shingle_array_msg)):
                    shingle_array_msg.append(3)
                else:
                    shingle_array_msg.append(0)
            elif shingle.shingle_status == ShingleStatus.PLACED:
                shingle_array_msg.append(1)
            elif shingle.shingle_status == ShingleStatus.INSTALLED:
                shingle_array_msg.append(2)
            else:
                shingle_array_msg.append(-1) # this should not happen, if it does there is an error some were
        roof_state.shingles = shingle_array_msg
        # second depot is the one on the right
        if len(self.shingle_depots) == 2:
            roof_state.depot_positions = [self.shingle_depots[0].get_location(), self.shingle_depots[1].get_location()]
        else:
            roof_state.depot_positions = [self.shingle_depots[0].get_location()]
        
        roof_state.header.stamp = rospy.Time.now()
        roof_state.inchworm_occ = sum(self.inchworm_occ, [])
        return roof_state

    def neighbor_is_placed(self, shingle_index):
        
        list_of_neighbors = []
        shingle_coord = [shingle_index%self.width, int(shingle_index/self.width)]
        # rospy.loginfo(f"the shingle index is {shingle_index}, for a coord {shingle_coord}")
        
        if shingle_coord[1]%2:
            for tuple in self.ODD_ROW_N_LOOKUP:
                neighbor_x = shingle_coord[0]+tuple[0]
                neighbor_y = shingle_coord[1]+tuple[1]
            
                if neighbor_x  >= 0 and neighbor_y >=0 and neighbor_x  < self.width and neighbor_y < self.height:
                    list_of_neighbors.append([shingle_coord[0]+tuple[0], shingle_coord[1]+tuple[1]])

        else:
            for tuple in self.EVEN_ROW_N_LOOKUP:
                neighbor_x = shingle_coord[0]+tuple[0]
                neighbor_y = shingle_coord[1]+tuple[1]

                if neighbor_x  >= 0 and neighbor_y >=0 and neighbor_x  < self.width and neighbor_y < self.height:
                    list_of_neighbors.append([shingle_coord[0]+tuple[0], shingle_coord[1]+tuple[1]])
        
        # rospy.logwarn(f"neightbors of {shingle_coord} is {list_of_neighbors}")

        for neighbor in list_of_neighbors:
            # rospy.loginfo(f"attempting to read at location {neighbor[1]} , {neighbor[0]}")

            if self.shingle_array[neighbor[1]][neighbor[0]] is not None:
                # rospy.logwarn (f"non-none tile at {neighbor[0]} , {neighbor[1]}, status is {self.shingle_array[neighbor[1]][neighbor[0]].shingle_status}, found by {shingle_coord}")
                if self.shingle_array[neighbor[1]][neighbor[0]].shingle_status == ShingleStatus.INSTALLED:
                    return True

        return False


    '''
    TODO:
        - show possible places to place a tile
    '''


   
                
