#!/usr/bin/env python3

import rospy, math

from assembly_msgs.msg import MateList
from assembly_msgs.srv import SuppressMate, SuppressMateRequest, SuppressLink, SuppressLinkRequest

from traj_planner import TrajectoryPlanner
from enum import Enum
from joint_consts import JointConstants

class Inchworm:
  '''
  High level controller for inchworms in the physics sim. Features:
  1. Manages mating and unmating with the magnet sim, including optimizations so that the sim runs at a reasonable speed
  2. swapFeet(): Swaps which foot is currently attached to the roof. Should be called when ready to swap which tile the inchworm is on.
  '''

  class Neighbors(Enum):
    NONE = 0 #for when the robot is straight up for motion planning, ignore for shingle
    UPPER_LEFT = 1
    UPPER_RIGHT = 2
    RIGHT = 3
    BOTTOM_RIGHT = 4
    BOTTOM_LEFT = 5
    LEFT = 6

  class Poses(Enum):
    STRAIGHT = 0
    RETRACT = 1
    UPPER_LEFT_DOWN = 2
    UPPER_LEFT_HOVER = 3
    UPPER_LEFT_LIFT = 4
    UPPER_RIGHT_DOWN = 5
    UPPER_RIGHT_HOVER = 6
    UPPER_RIGHT_LIFT = 7
    RIGHT_DOWN = 8
    RIGHT_HOVER = 9
    RIGHT_LIFT = 10
    BOTTOM_RIGHT_DOWN = 11
    BOTTOM_RIGHT_HOVER = 12
    BOTTOM_RIGHT_LIFT = 13
    BOTTOM_LEFT_DOWN = 14
    BOTTOM_LEFT_HOVER = 15
    BOTTOM_LEFT_LIFT = 16
    LEFT_DOWN = 17
    LEFT_HOVER = 18
    LEFT_LIFT = 19

  # Maps equivalent poses for swapping end effectors. If the upper end effector is down, then a lookup is done in this table to choose the correct pose.
  EE_POSE_MAP = {
    Poses.STRAIGHT: Poses.STRAIGHT,
    Poses.UPPER_LEFT_DOWN: Poses.BOTTOM_RIGHT_DOWN,
    Poses.UPPER_LEFT_HOVER: Poses.BOTTOM_RIGHT_HOVER,
    Poses.UPPER_LEFT_LIFT: Poses.BOTTOM_RIGHT_LIFT,
    Poses.UPPER_RIGHT_DOWN: Poses.BOTTOM_LEFT_DOWN,
    Poses.UPPER_RIGHT_HOVER: Poses.BOTTOM_LEFT_HOVER,
    Poses.UPPER_RIGHT_LIFT: Poses.BOTTOM_LEFT_LIFT,
    Poses.RIGHT_DOWN: Poses.LEFT_DOWN,
    Poses.RIGHT_HOVER: Poses.LEFT_HOVER,
    Poses.RIGHT_LIFT: Poses.LEFT_LIFT,
    Poses.BOTTOM_RIGHT_DOWN: Poses.UPPER_LEFT_DOWN,
    Poses.BOTTOM_RIGHT_HOVER: Poses.UPPER_LEFT_HOVER,
    Poses.BOTTOM_RIGHT_LIFT: Poses.UPPER_LEFT_LIFT,
    Poses.BOTTOM_LEFT_DOWN: Poses.UPPER_RIGHT_DOWN,
    Poses.BOTTOM_LEFT_HOVER: Poses.UPPER_RIGHT_HOVER,
    Poses.BOTTOM_LEFT_LIFT: Poses.UPPER_RIGHT_LIFT,
    Poses.LEFT_DOWN: Poses.RIGHT_DOWN,
    Poses.LEFT_HOVER: Poses.RIGHT_HOVER,
    Poses.LEFT_LIFT: Poses.RIGHT_LIFT
  }

  # Used to figure the direction of the previous neighbor. If the inchworm just went to Neighbors.LEFT, it came from Neighbors.RIGHT.
  LAST_NEIGHBOR_MAP = {
    Neighbors.LEFT: Neighbors.RIGHT,
    Neighbors.UPPER_LEFT: Neighbors.BOTTOM_RIGHT,
    Neighbors.UPPER_RIGHT: Neighbors.BOTTOM_LEFT,
    Neighbors.RIGHT: Neighbors.LEFT,
    Neighbors.BOTTOM_RIGHT: Neighbors.UPPER_LEFT,
    Neighbors.BOTTOM_LEFT: Neighbors.UPPER_RIGHT
  }

  # Poses needed to get to each neighbor position.
  NEIGHBOR_POSE_MAP = {
      Neighbors.BOTTOM_LEFT: [Poses.BOTTOM_LEFT_HOVER, Poses.BOTTOM_LEFT_LIFT, Poses.BOTTOM_LEFT_DOWN],
      Neighbors.BOTTOM_RIGHT:  [Poses.BOTTOM_RIGHT_HOVER, Poses.BOTTOM_RIGHT_LIFT, Poses.BOTTOM_RIGHT_DOWN],
      Neighbors.RIGHT:  [Poses.RIGHT_HOVER, Poses.RIGHT_LIFT, Poses.RIGHT_DOWN],
      Neighbors.UPPER_RIGHT:  [Poses.UPPER_RIGHT_HOVER, Poses.UPPER_RIGHT_LIFT, Poses.UPPER_RIGHT_DOWN],
      Neighbors.UPPER_LEFT:  [Poses.UPPER_LEFT_HOVER, Poses.UPPER_LEFT_LIFT, Poses.UPPER_LEFT_DOWN],
      Neighbors.LEFT:  [Poses.LEFT_HOVER, Poses.LEFT_LIFT, Poses.LEFT_DOWN],
      Neighbors.NONE: [Poses.STRAIGHT, Poses.STRAIGHT, Poses.STRAIGHT]
    }

  # Maps the Poses enum onto JointConstants (5-tuple of joint angles for a given pose) for bottom EE
  POSE_JOINT_MAP_BOTTOM = {
    Poses.STRAIGHT: JointConstants.BottomEE.straight,
    Poses.RETRACT: JointConstants.BottomEE.retract,

    Poses.UPPER_LEFT_HOVER: JointConstants.BottomEE.upper_left,
    Poses.UPPER_LEFT_DOWN: JointConstants.BottomEE.upper_left_down,
    Poses.UPPER_LEFT_LIFT: JointConstants.BottomEE.upper_left_lift,

    Poses.UPPER_RIGHT_HOVER: JointConstants.BottomEE.upper_right,
    Poses.UPPER_RIGHT_DOWN: JointConstants.BottomEE.upper_right_down,
    Poses.UPPER_RIGHT_LIFT: JointConstants.BottomEE.upper_right_lift,

    Poses.RIGHT_HOVER: JointConstants.BottomEE.right,
    Poses.RIGHT_DOWN: JointConstants.BottomEE.right_down,
    Poses.RIGHT_LIFT: JointConstants.BottomEE.right_lift,

    Poses.BOTTOM_RIGHT_HOVER: JointConstants.BottomEE.bottom_right,
    Poses.BOTTOM_RIGHT_DOWN: JointConstants.BottomEE.bottom_right_down,
    Poses.BOTTOM_RIGHT_LIFT: JointConstants.BottomEE.bottom_right_lift,

    Poses.BOTTOM_LEFT_HOVER: JointConstants.BottomEE.bottom_left,
    Poses.BOTTOM_LEFT_DOWN: JointConstants.BottomEE.bottom_left_down,
    Poses.BOTTOM_LEFT_LIFT: JointConstants.BottomEE.bottom_left_lift,

    Poses.LEFT_HOVER: JointConstants.BottomEE.left,
    Poses.LEFT_DOWN: JointConstants.BottomEE.left_down,
    Poses.LEFT_LIFT: JointConstants.BottomEE.left_lift
  
  }
  # Maps the Poses enum onto JointConstants (5-tuple of joint angles for a given pose) for top EE
  POSE_JOINT_MAP_TOP = {
    Poses.STRAIGHT: JointConstants.TopEE.straight,
    Poses.RETRACT: JointConstants.TopEE.retract,

    Poses.UPPER_LEFT_HOVER: JointConstants.TopEE.upper_left,
    Poses.UPPER_LEFT_DOWN: JointConstants.TopEE.upper_left_down,
    Poses.UPPER_LEFT_LIFT: JointConstants.TopEE.upper_left_lift,

    Poses.UPPER_RIGHT_HOVER: JointConstants.TopEE.upper_right,
    Poses.UPPER_RIGHT_DOWN: JointConstants.TopEE.upper_right_down,
    Poses.UPPER_RIGHT_LIFT: JointConstants.TopEE.upper_right_lift,

    Poses.RIGHT_HOVER: JointConstants.TopEE.right,
    Poses.RIGHT_DOWN: JointConstants.TopEE.right_down,
    Poses.RIGHT_LIFT: JointConstants.TopEE.right_lift,

    Poses.BOTTOM_RIGHT_HOVER: JointConstants.TopEE.bottom_right,
    Poses.BOTTOM_RIGHT_DOWN: JointConstants.TopEE.bottom_right_down,
    Poses.BOTTOM_RIGHT_LIFT: JointConstants.TopEE.bottom_right_lift,

    Poses.BOTTOM_LEFT_HOVER: JointConstants.TopEE.bottom_left,
    Poses.BOTTOM_LEFT_DOWN: JointConstants.TopEE.bottom_left_down,
    Poses.BOTTOM_LEFT_LIFT: JointConstants.TopEE.bottom_left_lift,

    Poses.LEFT_HOVER: JointConstants.TopEE.left,
    Poses.LEFT_DOWN: JointConstants.TopEE.left_down,
    Poses.LEFT_LIFT: JointConstants.TopEE.left_lift
  
  }

  # Maps relative even-r coords to a neighbors
  EVEN_ROW_LOOKUP_NEIGHBOR_MAP = {
    (1, 0): Neighbors.RIGHT,
    (1, -1): Neighbors.BOTTOM_RIGHT,
    (0, -1): Neighbors.BOTTOM_LEFT,
    (-1, 0): Neighbors.LEFT,
    (0, 1): Neighbors.UPPER_LEFT,
    (1, 1): Neighbors.UPPER_RIGHT
  }
  ODD_ROW_LOOKUP_NEIGHBOR_MAP = {
    (1, 0): Neighbors.RIGHT,
    (0, -1): Neighbors.BOTTOM_RIGHT,
    (-1, -1): Neighbors.BOTTOM_LEFT,
    (-1, 0): Neighbors.LEFT,
    (-1, 1): Neighbors.UPPER_LEFT,
    (0, 1): Neighbors.UPPER_RIGHT
  }

  #                    right   l right  l left   left    up left  up right
  EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
  ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

  def __init__(self, idx=0):
    self.idx = idx

    self.planner = TrajectoryPlanner(idx=self.idx)

    self.roof_height = rospy.get_param("/roof_height")
    self.roof_width  = rospy.get_param("/roof_width")

    # 0 is bottom foot, 1 is top foot
    self.foot_down = 0

    # Index of shingle the robot is on. Updated by mateCB from Magnet Sim.
    self.on_shingle = idx * 2

    # Whether the inchworm is currently holding a shingle
    self.holdingShingle = False

    #for higher abstraction, last neighbor sent
    self.lastNeighbor = Inchworm.Neighbors.NONE
    # The roof coordinate that the robot is currently on. Roof idx != shingle idx. Updated by mateCB
    self.on_coord = self.idx_to_coord(self.on_shingle)

    # Most recent mate callback. Used to inspect current shingle config
    self.last_mate_msg = None

    # Suppression service proxy
    rospy.wait_for_service("/suppress_mate")
    self.mate_suppress_proxy = rospy.ServiceProxy("/suppress_mate", SuppressMate)
    rospy.wait_for_service("/suppress_link")
    self.link_suppress_proxy = rospy.ServiceProxy("/suppress_link", SuppressLink)

    # State subscribers
    rospy.Subscriber("/active_mates", MateList, self.mateCB)

    rospy.sleep(0.25)

    # self.mag_state_pub = rospy.Publisher()

    rospy.loginfo("Initializing link mates...")
    self.initializeMates()

    rospy.loginfo(f"Initialize inchworm class for inchworm {self.idx}.")

  ###################
  ###   HELPERS   ###
  ###################
  def idx_to_coord(self, index):
    return (index % self.roof_width, math.floor(index / self.roof_width))

  def coord_to_idx(self, coord):
    return self.roof_width*coord[1] + coord[0]

  def getAdjacentShingleIndexes(self, roof_coord):
    '''
    Returns a list of shingle indexes that are adjacent to the robot. Used to determine which mates to disable
    This function uses self.on_coord, looks up adjacent roof indexes, then finds shingles that are attached to those roof mount points.
    '''
    #                    right   l right  l left   left    up left  up right
    EVEN_ROW_N_LOOKUP = [(1, 0), (1, -1), (0, -1), (-1, 0), (0, 1), (1, 1)]
    ODD_ROW_N_LOOKUP = [(1, 0), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1)]

    # The coordinate we look up relative to is the roof coordinate that the inchworm is on

    pair_offsets = []

    # Determine which list to lookup from, based on whether this inchworm is on an even or odd row
    if roof_coord[1] % 2 == 0:
      pair_offsets = EVEN_ROW_N_LOOKUP
    else:
      pair_offsets = ODD_ROW_N_LOOKUP

    neighbor_coords = []

    # If the coordinate is in bounds, append it to the list
    for pair in pair_offsets:
      adj_coord = (roof_coord[0] + pair[0], roof_coord[1] + pair[1])
      if adj_coord[0] >= 0 and adj_coord[0] < self.roof_width and adj_coord[1] >= 0 and adj_coord[1] < self.roof_height:
        neighbor_coords.append(adj_coord)

    # Convert all adjacent coordinates into indexes, so that we can lookup into the mate message
    roof_indexes = [self.coord_to_idx(coord) for coord in neighbor_coords]

    # Shingle indexes attached to the roof_indexes above
    shingle_indexes = []

    while self.last_mate_msg is None:
      rospy.sleep(0.1)

    last_mate = self.last_mate_msg
    for idx in roof_indexes:
      for i,mate in enumerate(last_mate.male):
        # If we find the corresponding roof mate point in the male list, lookup the shingle index and add it to the output list
        if idx == int(mate.split("::")[-1]):
          shingle_idx = int(last_mate.female[i].split("::")[1].split("_")[-1])
          shingle_indexes.append(shingle_idx)

    rospy.loginfo(f"Shingle indexes adjacent to roof coord {roof_coord}:")
    for idx in shingle_indexes:
      rospy.loginfo(f"\t{idx}")

    return shingle_indexes

  def absoluteToNeighbor(self, roof_coord):
    # Offset to get from where inchworm is to the desired position
    coord_offset = (roof_coord[0] - self.on_coord[0], roof_coord[1] - self.on_coord[1])

    neighbor = None

    # Determine the neighbor to move to
    if self.on_coord[1] % 2 == 0:
      neighbor = Inchworm.EVEN_ROW_LOOKUP_NEIGHBOR_MAP[coord_offset]
    else:
      neighbor = Inchworm.ODD_ROW_LOOKUP_NEIGHBOR_MAP[coord_offset]

    return neighbor

  def isInchwormOnRoof(self):
    '''
    Determines whether the inchworm is currently attached to the roof. There are two cases where this function will return False:
      1. The inchworm is in the middle of a step, and has both end effectors disconnected.
      2. The inchworm has fallen off of the roof, or is in the process of doing so.

    for all mates:
      if the inchworm shows up in the mate list:
        find the shingle its attached to
        if the shingle is attached to the roof:
          return True
    return False
    '''

    last_mate = self.last_mate_msg

    for i,iw_mate in enumerate(last_mate.male):
      if f"inchworm_description_{self.idx}" in iw_mate:
        on_shingle = int(last_mate.female[i].split("::")[1].split("_")[-1])

        for shingle_mate in last_mate.female:
          if f"shingle_description_{on_shingle}::" in shingle_mate:
            return True

    return False

  #########################
  ### MAGNET MANAGEMENT ###
  #########################
  def initializeMates(self):
    '''
    Run on __init__(). Deactivates all inchworm to shingle mates, then reenables just the bottom link.
    '''

    # Scoped inchworm links
    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]

    # Suppress both links, which will disable all shingle mates.
    req = SuppressLinkRequest()
    req.suppress = True
    req.scoped_link = iw_bot

    self.link_suppress_proxy(req)
    
    req.scoped_link = iw_top
    self.link_suppress_proxy(req)

    # Enable iw_root -> shingle_{on_coord}
    req = SuppressMateRequest()
    req.suppress = False
    req.scoped_male = iw_bot
    req.scoped_female = ["inchworm", f"shingle_description_{self.on_shingle}", f"shingle_{self.on_shingle}"]

    self.mate_suppress_proxy(req)

  def swapMagnet(self, turnOff, turnOn, neighbor):
    req = SuppressLinkRequest()
    req.scoped_link = turnOff
    req.suppress = True

    self.link_suppress_proxy(req)

    # Calculate neighbor shingle index
    if self.on_coord[1] % 2 == 0:
      inverted = {v: k for k, v in Inchworm.EVEN_ROW_LOOKUP_NEIGHBOR_MAP.items()}
    else:
      inverted = {v: k for k, v in Inchworm.ODD_ROW_LOOKUP_NEIGHBOR_MAP.items()}

    neighbor_offset = inverted[neighbor]
    going_to = (self.on_coord[0] + neighbor_offset[0], self.on_coord[1] + neighbor_offset[1])
    going_to_roof_idx = self.coord_to_idx(going_to)

    rospy.loginfo(f"Going to shingle coordinate {going_to}")
    shingle_idx = self.roofToShingle(going_to_roof_idx)
    rospy.loginfo(self.last_mate_msg.male)
    rospy.loginfo(self.last_mate_msg.female)
    rospy.loginfo(f"Which is shingle index {shingle_idx}")

    req = SuppressMateRequest()
    req.scoped_male = turnOn
    req.scoped_female = ["inchworm", f"shingle_description_{shingle_idx}", f"shingle_{shingle_idx}"]
    req.suppress = False

    self.mate_suppress_proxy(req)

    rospy.sleep(0.25)

  def mateCB(self, msg):
    '''
    Callback for active_mate messages. Used to determine where the robot is currently
    '''

    self.last_mate_msg = msg
    
    # For all iw_root_N in the mates
    for i,mate in enumerate(msg.male):
      # If my index is in the string, we're mated to a shingle
      if f"inchworm_description_{self.idx}" in mate:

        # Fully scoped shingle names come in this format:
        # inchworm::shingle_description_{idx}::shingle_{idx}::1
        # Following line is to extract the first idx.
        on_shingle = int(msg.female[i].split("::")[1].split("_")[-1])

        # If the robot has moved, trigger an update on suppressed mates
        if not on_shingle == self.on_shingle:

          # Check if shingle is attached to roof
          for j,mate in enumerate(msg.female):
            if f"shingle_description_{on_shingle}::" in mate:
              male_mate = msg.male[j]
              if f"roof_description_0" in male_mate:
                # Shingle is on the roof. Update on_shingle and on_coord
                rospy.loginfo(f"inchworm_{self.idx} was on shingle_{self.on_shingle}, now on shingle_{on_shingle}")
                rospy.loginfo(mate)
                rospy.loginfo(male_mate)
                self.on_shingle = on_shingle
                roof_mate_idx = int(msg.male[j].split("::")[-1])
                self.on_coord = self.idx_to_coord(roof_mate_idx)
                rospy.loginfo(f"inchworm_{self.idx} on index {roof_mate_idx}, coord {self.on_coord}")

                return

  ################
  ### MOVEMENT ###
  ################
  def moveTo(self,pose,time):
    '''
    Moves the end effector not currently attached to the roof to a neighbor.
    neighbor: One of the `Neighbors` enum. This gets mapped to a JointConstants set of joint angles
    '''

    print(f"going to: {pose}")
    #newPose = pose # if foot down is 0
    #if self.foot_down == 1: #if foot down is 1, do the mapping
    #  newPose = Inchworm.EE_POSE_MAP.get(pose)
    #  print(f"treated as {newPose}")

    angles = []

    if self.foot_down == 0:
      angles = Inchworm.POSE_JOINT_MAP_BOTTOM.get(pose)

    elif self.foot_down == 1:
      angles = Inchworm.POSE_JOINT_MAP_TOP.get(pose)

    self.planner.run_quintic_traj(angles, time)

  def retract(self):
    '''
    Will retract the inchworm to the spin position.
    '''

    angles = self.planner.get_joint_state()

    if self.foot_down == 0:
      angles[1:4] = Inchworm.POSE_JOINT_MAP_BOTTOM.get(Inchworm.Poses.RETRACT)[1:4]

    elif self.foot_down == 1:
      angles[1:4] = Inchworm.POSE_JOINT_MAP_TOP.get(Inchworm.Poses.RETRACT)[1:4]

    print(angles)

    self.planner.run_quintic_traj(angles, 5.0)

  def spinTo(self, neighbor):
    '''
    Spins the robot (while in the retract position) to point at a neighbor
    '''

    angles = self.planner.get_joint_state()

    desired_angles = []
    duration = 0

    # We will travel 2pi radians in this time duration.
    DURATION_SCALAR = 10

    if self.foot_down == 0:
      desired_angles = Inchworm.POSE_JOINT_MAP_BOTTOM.get(Inchworm.NEIGHBOR_POSE_MAP.get(neighbor)[0])
      print(f"Delta distance: {abs(angles[0] - desired_angles[0])} rad")
      duration = (abs(angles[0] - desired_angles[0]) / (2*math.pi)) * DURATION_SCALAR
    elif self.foot_down == 1:
      desired_angles = Inchworm.POSE_JOINT_MAP_TOP.get(Inchworm.NEIGHBOR_POSE_MAP.get(neighbor)[0])
      print(f"Delta distance: {abs(angles[0] - desired_angles[0])} rad")
      duration = (abs(angles[4] - desired_angles[4]) / (2*math.pi)) * DURATION_SCALAR


    angles[0], angles[4] = desired_angles[0], desired_angles[4]

    self.planner.run_quintic_traj(angles, duration)

  def move(self, neighbor, plantFoot=True):
    '''
    Moves the robot to an adjacent shingle. Will swap feet if plantFoot, unless neighbor is NONE (robot stands straight up)
    '''
    #if the inchworm is not straight up and down, move up from last position
    if (self.lastNeighbor != Inchworm.Neighbors.NONE):
      poses = Inchworm.NEIGHBOR_POSE_MAP.get(self.lastNeighbor)
      
      if not self.holdingShingle:
        # Move to spin position
        self.retract()
        rospy.sleep(1.0)
      else:
        poses = Inchworm.NEIGHBOR_POSE_MAP.get(self.lastNeighbor)
        #hover above last place
        self.moveTo(poses[0], 3.0)
        #lift above last place
        self.moveTo(poses[1], 3.0)
        rospy.sleep(1.0)

    
    poses = Inchworm.NEIGHBOR_POSE_MAP.get(neighbor)
    print(f"NEIGHBOR: {neighbor}")
    
    # Point at neighbor
    self.spinTo(neighbor)
    rospy.sleep(1.0)

    #go to lift above next place
    self.moveTo(poses[1], 3.0)
    #hover above next place
    self.moveTo(poses[0], 2.0)
    #plant foot
    self.moveTo(poses[2], 2.0)

    self.lastNeighbor = neighbor

    if ((plantFoot) and (neighbor != Inchworm.Neighbors.NONE)):
      self.swapFeet()
    
    rospy.sleep(.1)

  def swapFeet(self):
    '''
    Swaps which foot is attached to the roof. Make sure before calling the free end effector is placed above a mounting point before calling.
    '''
    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]    

    rospy.loginfo(f"foot down: {self.foot_down}")
    rospy.loginfo(f"on_shingle: {self.on_shingle}")

    if (self.foot_down == 0):
      turnOff = iw_bot
      turnOn = iw_top
      #switch to top foot
      foot_down = 1

    elif (self.foot_down == 1):
      turnOff = iw_top
      turnOn = iw_bot
      #switch to bottom foot
      foot_down = 0

    self.foot_down = foot_down
    #swap where the last neighbor was
    temp = self.lastNeighbor
    self.lastNeighbor = Inchworm.LAST_NEIGHBOR_MAP.get(temp)
    self.swapMagnet(turnOff, turnOn, temp)

    rospy.sleep(1.5)

  ################
  ### SHINGLES ###
  ################
  def getAttachedShingle(self, end_effector):
    '''
    Returns the shingle index attached to end_effector (bottom if 0, top if 1), or -1 if that EE has no shingle attached.
    '''

    ee_string = "iw_foot_top" if end_effector else "iw_root"

    last_mate = self.last_mate_msg
    print(last_mate)

    print(f"inchworm_description_{self.idx}::{ee_string}_{self.idx}")

    for i,mate in enumerate(last_mate.male):
      if f"inchworm_description_{self.idx}::{ee_string}" in mate:
        return int(last_mate.female[i].split("::")[-2].split("_")[-1])

    return -1

  def roofToShingle(self, roof_idx):
    '''
    Finds the shingle attached to a roof index.
    '''

    # Shingle indexes attached to the roof_indexes above
    shingle_idx = None

    while self.last_mate_msg is None:
      rospy.sleep(0.1)

    last_mate = self.last_mate_msg
    for i,mate in enumerate(last_mate.male):
      # If we're not looking at the temporary roof, and the roof index matches, lookup the shingle index and add it to the output list
      if mate.split("::")[1][-1] != "1" and roof_idx == int(mate.split("::")[-1]):
        shingle_idx = int(last_mate.female[i].split("::")[1].split("_")[-1])
        return shingle_idx

  def shingleToRoof(self, shingle_idx):
    '''
    Inverse of roofToShingle. Maps a shingle index to a roof index.
    '''

    roof_idx = None

    while self.last_mate_msg is None:
      rospy.sleep(0.1)

    last_mate = self.last_mate_msg
    for i,mate in enumerate(last_mate.female):
      if mate.split("::")[-2] == f"shingle_{shingle_idx}":
        roof_idx = int(last_mate.male[i].split("::")[-1])
        return roof_idx

  def suppressShingle(self, shingle_idx):
    shingle = ["inchworm", f"shingle_description_{shingle_idx}", f"shingle_{shingle_idx}"]
    roof = ["inchworm", f"roof_description_0", f"roof_0"]
    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]
    inchworm = []
    if (self.foot_down == 0):
      inchworm = iw_top
    else:
      inchworm = iw_bot
    
    rospy.loginfo(f"\tSuppressing shingle {shingle_idx}")

    #suppress shingle to roof
    req = SuppressMateRequest()
    req.suppress = True
    req.scoped_female = shingle
    req.scoped_male = roof
    self.mate_suppress_proxy(req)

    #unsupress shingle to inchworm
    req.suppress = False
    req.scoped_female = shingle
    req.scoped_male = inchworm
    rospy.loginfo(f"\tUnsuppressing shingle {shingle_idx}")
    self.mate_suppress_proxy(req)

  def unsupressShingle(self, shingle_idx):
    shingle = ["inchworm", f"shingle_description_{shingle_idx}", f"shingle_{shingle_idx}"]
    roof = ["inchworm", f"roof_description_0", f"roof_0"]
    iw_bot = ["inchworm", f"inchworm_description_{self.idx}", f"iw_root_{self.idx}"]
    iw_top = ["inchworm", f"inchworm_description_{self.idx}", f"iw_foot_top_{self.idx}"]
    inchworm = []
    if (self.foot_down == 0):
      inchworm = iw_top
    else:
      inchworm = iw_bot
    
    rospy.loginfo(f"\tSuppressing shingle {shingle_idx}")

    
    req = SuppressMateRequest()
    req.suppress = True
    

    #supress shingle to inchworm
    req.scoped_female = shingle
    req.scoped_male = inchworm
    self.mate_suppress_proxy(req)

    #unsuppress shingle to roof
    req.suppress = False
    req.scoped_female = shingle
    req.scoped_male = roof
    rospy.loginfo(f"\tUnsuppressing shingle {shingle_idx}")
    self.mate_suppress_proxy(req)

  def getNeighborCoord(self, neighbor):
    pair_offsets = []

    # Determine which list to lookup from, based on whether this inchworm is on an even or odd row
    if self.on_coord[1] % 2 == 0:
      pair_offsets = Inchworm.EVEN_ROW_N_LOOKUP
    else:
      pair_offsets = Inchworm.ODD_ROW_N_LOOKUP
    
    pair = None
    
    if (neighbor == Inchworm.Neighbors.LEFT):
      pair = pair_offsets[3]
    elif (neighbor == Inchworm.Neighbors.UPPER_LEFT):
      pair = pair_offsets[4]
    elif (neighbor == Inchworm.Neighbors.UPPER_RIGHT):
      pair = pair_offsets[5] 
    elif (neighbor == Inchworm.Neighbors.RIGHT):
      pair = pair_offsets[0]
    else:
      rospy.logerr(f"Unsuppored neighbor ({neighbor}) to get relative coord for")

    return pair

  def pickupShingle(self, neighbor):
    rospy.sleep(1)
    #make sure robot doesn't grab a supporting shingle
    if ((neighbor == Inchworm.Neighbors.BOTTOM_LEFT) or (neighbor == Inchworm.Neighbors.BOTTOM_RIGHT)):
      print("Error: cannot grab shingle underneath the shingle foot is planted on")
      return

    pair = self.getNeighborCoord(neighbor)
      
    #grab the coord of the shingle you're going to grab
    roof_coord = [self.on_coord[0] + pair[0], self.on_coord[1] + pair[1]]
    roof_idx = self.coord_to_idx(roof_coord)
    shingle_idx = self.roofToShingle(roof_idx)

    # return if there's no shingle there
    #move to that shingle
    #self.move(neighbor, False)
    #pop that shingle off the roof
    self.suppressShingle(shingle_idx)
    #pull up a bit
  
    self.holdingShingle = True

  def placeShingle(self, neighbor):
    #make sure robot doesn't try to place in supporting shingle place
    if ((neighbor == Inchworm.Neighbors.BOTTOM_LEFT) or (neighbor == Inchworm.Neighbors.BOTTOM_RIGHT)):
      print("Error: cannot place shingle underneath the shingle foot is planted on")
      return
    
    pair = self.getNeighborCoord(neighbor)
    print(f"unsupress pair: {pair}")
    #grab the coord of the roof index you're going to place on
    roof_coord = [self.on_coord[0] + pair[0], self.on_coord[1] + pair[1]]
    roof_idx = self.coord_to_idx(roof_coord)

    print(f"last neighbor: {self.lastNeighbor}, currNeighbor: {neighbor}")
    #move shingle to place point

    #pull up a bit
    poses = Inchworm.NEIGHBOR_POSE_MAP.get(neighbor)

    # magnetize shingle on the end effector not currently on the roof and disconnect from robot
    self.unsupressShingle(self.getAttachedShingle((self.foot_down + 1) % 2))

    self.holdingShingle = False

  def unitTest(self):
    rospy.loginfo("check right")
    self.move(Inchworm.Neighbors.RIGHT, plantFoot=False)
    rospy.loginfo("check")
    rospy.sleep(5)
    rospy.loginfo("check upper right")
    self.move(Inchworm.Neighbors.UPPER_RIGHT, plantFoot=False)
    rospy.loginfo("check")
    rospy.sleep(5)
    rospy.loginfo("check upper left")
    self.move(Inchworm.Neighbors.UPPER_LEFT, plantFoot=False)
    rospy.loginfo("check")
    rospy.sleep(5)
    rospy.loginfo("check leftt")
    self.move(Inchworm.Neighbors.LEFT, plantFoot=False)
    rospy.sleep(5)
    rospy.loginfo("check lower left")
    self.move(Inchworm.Neighbors.BOTTOM_LEFT, plantFoot=False)
    rospy.loginfo("check")
    rospy.sleep(5)
    rospy.loginfo("check lower right")
    self.move(Inchworm.Neighbors.BOTTOM_RIGHT, plantFoot=False)
    rospy.loginfo("check")
    rospy.sleep(5)

  def walkingAround(self):
    rospy.loginfo("go right")
    self.move(Inchworm.Neighbors.RIGHT)
    rospy.loginfo("go upper left")
    self.move(Inchworm.Neighbors.UPPER_LEFT)
    rospy.loginfo("go left")
    self.move(Inchworm.Neighbors.LEFT)
    rospy.loginfo("go lower left")
    self.move(Inchworm.Neighbors.BOTTOM_LEFT)
    rospy.loginfo("go lower right")
    self.move(Inchworm.Neighbors.BOTTOM_RIGHT)
    rospy.loginfo("go upper right")
    self.move(Inchworm.Neighbors.UPPER_RIGHT)

  def hexagon(self):
    rospy.loginfo("go lower right")
    self.move(Inchworm.Neighbors.BOTTOM_RIGHT)
    rospy.sleep(1)
    rospy.loginfo("go right")
    self.move(Inchworm.Neighbors.RIGHT)
    rospy.sleep(1)
    rospy.loginfo("go upper right")
    self.move(Inchworm.Neighbors.UPPER_RIGHT)
    rospy.sleep(1)
    rospy.loginfo("go upper left")
    self.move(Inchworm.Neighbors.UPPER_LEFT)
    rospy.sleep(1)
    rospy.loginfo("go left")
    self.move(Inchworm.Neighbors.LEFT)
    rospy.sleep(1)
    rospy.loginfo("go lower left")
    self.move(Inchworm.Neighbors.BOTTOM_LEFT)
    rospy.sleep(1)
    rospy.loginfo("straighten")
    self.move(Inchworm.Neighbors.NONE)

def difference(list1, list2):
  diff = [(list1[0] - list2[0]), (list1[1] - list2[1]), (list1[2] - list2[2]), (list1[3] - list2[3]), (list1[4] - list2[4])]
  return diff

def add(list1, list2):
  diff = [(list1[0] + list2[0]), (list1[1] + list2[1]), (list1[2] + list2[2]), (list1[3] + list2[3]), (list1[4] + list2[4])]
  return diff

def shingleEvenRow(row):
  # Place a shingle in each spot
  for i in range(5):
    manager.spawnShingle((0, row))
    rospy.sleep(0.5)
    for j in range(4-i):
      iw.move(Inchworm.Neighbors.UPPER_LEFT, plantFoot=False)
      rospy.sleep(0.5)
      iw.pickupShingle(Inchworm.Neighbors.UPPER_LEFT)
      rospy.sleep(0.5)
      iw.move(Inchworm.Neighbors.UPPER_RIGHT, plantFoot=False)
      rospy.sleep(0.5)
      iw.placeShingle(Inchworm.Neighbors.UPPER_RIGHT)
      rospy.sleep(0.5)

      if j != (4-i-1):
        iw.move(Inchworm.Neighbors.RIGHT)
        rospy.sleep(0.5)
    for j in range(3-i):
      iw.move(Inchworm.Neighbors.LEFT)

def shingleOddRow(row):
  # Place a shingle in each spot
  for i in range(5):
    manager.spawnShingle((0, row))
    rospy.sleep(0.5)
    for j in range(4-i):
      iw.move(Inchworm.Neighbors.UPPER_LEFT, plantFoot=False)
      rospy.sleep(0.5)
      iw.pickupShingle(Inchworm.Neighbors.UPPER_LEFT)
      rospy.sleep(0.5)
      iw.move(Inchworm.Neighbors.UPPER_RIGHT, plantFoot=False)
      rospy.sleep(0.5)
      iw.placeShingle(Inchworm.Neighbors.UPPER_RIGHT)
      rospy.sleep(0.5)

      if j != (4-i-1):
        iw.move(Inchworm.Neighbors.RIGHT)
        rospy.sleep(0.5)
    for j in range(3-i):
      iw.move(Inchworm.Neighbors.LEFT)

if __name__ == "__main__":
  # This only exists to test the class.

  from shingle_manager import ShingleManager

  rospy.init_node("iw_class_test")

  iw = Inchworm(idx=0)
  rospy.sleep(1)

  manager = ShingleManager(rospy.get_param("/roof_width"), rospy.get_param("/roof_height"))

  input()

  # iw.move(Inchworm.Neighbors.RIGHT)
  # iw.move(Inchworm.Neighbors.RIGHT)
  # iw.move(Inchworm.Neighbors.RIGHT)
  for row in range(1,4):
    for i in range(5):
      manager.spawnShingle((i, row))
  #for row in range(4):
  #  if (row+1) % 2 == 0:
  #    shingleEvenRow(row+1)
  #    iw.move(Inchworm.Neighbors.UPPER_LEFT)
  #  else:
  #    shingleOddRow(row+1)
  #    iw.move(Inchworm.Neighbors.UPPER_RIGHT)