from enum import Enum

from inchworm_algo.msg import ShingleMsg
import rospy
# all x and y are in array coords currently


class EdgeStatus(Enum):
    NO_EDGE = 0
    RIGHT = 1
    LEFT = 2
    BOTTEM = 3
    TOP = 4
    BOTTEM_RIGHT = 5
    BOTTEM_LEFT = 6
    TOP_RIGHT = 7
    TOP_LEFT = 8

class NeighborIndex(Enum):
    RIGHT = 0
    BOTTEM_RIGHT = 1
    BOTTEM_LEFT = 2
    LEFT = 3
    TOP_LEFT = 4
    TOP_RIGHT = 5


class ShingleStatus(Enum):
    UNINSTALLED = 0
    PLACED = 1
    INSTALLED = 2

class ShingleType(Enum):
    NORMAL_SHINGLE = 0
    HALF_SHINGLE = 1

class Shingle():

    id = -1
    x_coord = -1
    y_coord = -1
    on_frontier = False
    edge = EdgeStatus.NO_EDGE
    is_half_shingle = False

    shingle_status = ShingleStatus.UNINSTALLED

    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]
    

    def __init__(self, id = -1, is_half_shingle = False):
        self.id = id # this value should not change once it is assigned
        self.on_frontier = False
        self.is_half_shingle = is_half_shingle
        self.shingle_status = ShingleStatus.PLACED ## TODO: THIS IS MAGIC TO MAKE THE ALGO SIM WORK FOR NOW
        self.neighbors_status = [ShingleStatus.UNINSTALLED] * 6
        self.neighbors_update_times = [0] * 6
        self.most_recent_target = 0



    def create_from_message(self, msg):
        self.id = msg.id
        self.is_placed = msg.is_placed
        self.x_coord = msg.x_coord
        self.y_coord = msg.y_coord
        self.neighbors_ids = msg.neighbors_ids
        self.neighbors_status = msg.neighbors_status
        self.on_frontier = msg.on_frontier
        self.is_half_shingle = msg.is_half_shingle
        self.shingle_status = msg.shingle_status
        return self



    def place_shingle(self, x, y,):
        self.x_coord = x
        self.y_coord = y
        self.shingle_status = ShingleStatus.PLACED
        return self

    def pickup_shingle(self):
        self.x_coord = -1
        self.y_coord = -1
        self.shingle_status = ShingleStatus.UNINSTALLED
        return self
        

    
    def install_shingle(self, x, y):
        self.x_coord = x
        self.y_coord = y
        self.on_frontier = True
        self.shingle_status = ShingleStatus.INSTALLED

        return self


    # get ids of all the neighbors and have the roof update the status 
    # n_location is a NeighborIndex
    # honestly not sure if we want to use this but it should only be used by the robot
    def update_neighbor(self, n_locatation, n_status):
        if self.y_coord %2 == 0:
            neighbor_location = (self.x_coord + self.EVEN_ROW_N_LOOKUP[n_locatation][0], self.y_coord + self.EVEN_ROW_N_LOOKUP[n_locatation][1])
        else:
            neighbor_location = (self.x_coord + self.ODD_ROW_N_LOOKUP[n_locatation][0], self.y_coord + self.ODD_ROW_N_LOOKUP[n_locatation][1])
        
        if self.neighbors_status[n_locatation] != ShingleStatus.INSTALLED:
            self.neighbors_status[n_locatation] = n_status
        

    def get_neighbor_locations_and_status(self):
        location_status = {}
        for n in range(6):
            if self.y_coord %2 == 0:
                neighbor_location = (self.x_coord + self.EVEN_ROW_N_LOOKUP[n][0], self.y_coord + self.EVEN_ROW_N_LOOKUP[n][1])
            else:
                neighbor_location = (self.x_coord + self.ODD_ROW_N_LOOKUP[n][0], self.y_coord + self.ODD_ROW_N_LOOKUP[n][1])
            location_status[neighbor_location] = self.neighbors_status[n]
        return location_status

    def convert_to_neighbor_index(self, coord):
        for n in range(6):
            if self.y_coord %2 == 0:
                neighbor_location = (self.x_coord + self.EVEN_ROW_N_LOOKUP[n][0], self.y_coord + self.EVEN_ROW_N_LOOKUP[n][1])
            else:
                neighbor_location = (self.x_coord + self.ODD_ROW_N_LOOKUP[n][0], self.y_coord + self.ODD_ROW_N_LOOKUP[n][1])
            if neighbor_location == coord:
                return n



    def to_message(self):
        msg = ShingleMsg()
        msg.id = self.id
        msg.x_coord = self.x_coord
        msg.y_coord = self.y_coord
        msg.neighbors_ids = self.neighbors_ids
        msg.neighbors_status = self.neighbors_status
        msg.on_frontier = self.on_frontier
        msg.is_half_shingle = self.is_half_shingle
        msg.shingle_status = self.shingle_status
        return msg
        