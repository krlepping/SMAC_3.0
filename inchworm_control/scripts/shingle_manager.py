#!/usr/bin/env python3

import rospy, math, tf2_ros

from tf.transformations import quaternion_from_euler

from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState, GetModelStateRequest
from assembly_msgs.msg import MateList
from assembly_msgs.srv import SuppressMatePair, SuppressMatePairRequest, SuppressMate, SuppressMateRequest, SuppressLink, SuppressLinkRequest
from std_srvs.srv import Empty, EmptyRequest

class ShingleManager():
  def __init__(self, roof_width, roof_height):
    self.roof_width  = roof_width
    self.roof_height = roof_height
    self.shingle_count = roof_width * roof_height

    # How many shingles moved to the roof so far. Starts at roof_width since the first row starts on the roof
    self.shingles_moved = roof_width

    rospy.wait_for_service("/gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/get_model_state")
    self.gazebo_move_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    self.gazebo_get_service = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    rospy.wait_for_service("/suppress_mate_pair")
    rospy.wait_for_service("/suppress_mate")
    rospy.wait_for_service("/suppress_link")
    self.mate_pair_suppress_proxy = rospy.ServiceProxy("/suppress_mate_pair", SuppressMatePair)
    self.mate_suppress_proxy = rospy.ServiceProxy("/suppress_mate", SuppressMate)
    self.link_suppress_proxy = rospy.ServiceProxy("/suppress_link", SuppressLink)
    
    self.buffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.buffer)
    self.initializeMates()

  def initializeMates(self):

    # Suppress all shingles to everything
    req = SuppressLinkRequest()
    req.suppress = True
    for i in range(self.shingle_count):
      req.scoped_link = ["inchworm", f"shingle_description_{i}", f"shingle_{i}"]

      self.link_suppress_proxy(req)

    # Cycle through the shingles on roof 0, turn on specific mate point
    req = SuppressMatePairRequest()
    req.suppress = False
    req.scoped_male = ["inchworm", f"roof_description_0", f"roof_0"]
    req.female_mate_id = 1

    for i in range (self.roof_width):
      req.scoped_female = ["inchworm", f"shingle_description_{i}", f"shingle_{i}"]

      req.male_mate_id = i

      rospy.loginfo(f"suppress shingle {i} to roof 1")
      self.mate_pair_suppress_proxy(req)
    
    req.scoped_male = ["inchworm", f"roof_description_1", f"roof_1"] #don't think I can get more specific than this
    # Cycle through the shingles on roof 1, turn on specific mate point
    for i in range(self.roof_width, self.shingle_count):
      req.scoped_female = ["inchworm", f"shingle_description_{i}", f"shingle_{i}"]

      req.male_mate_id = i

      rospy.loginfo(f"suppress shingle {i} to roof 0")
      self.mate_pair_suppress_proxy(req)
    
  def suppressShingle(self, idx):

    female = ["inchworm", f"shingle_description_{idx}", f"shingle_{idx}"]
    male = ["inchworm", f"roof_description_1", "roof_1"]

    req = SuppressMateRequest()
    req.suppress = True
    req.scoped_female = female
    req.scoped_male = male

    self.mate_suppress_proxy(req)

    male = ["inchworm", f"roof_description_0", "roof_0"]
    req.suppress = False
    req.scoped_male = male

    self.mate_suppress_proxy(req)

  def spawnShingle(self, roof_coord):
    rospy.loginfo(f"Spawning shingle at {roof_coord}")

    # Begin by suppressing the shingle on roof 1, then delay to make sure the suppress went through
    self.suppressShingle(self.shingles_moved)
    rospy.sleep(0.25)

    # Shingle placement constants. MUST match what is in inchworm_description/sdf/all_models.sdf
    # (except for Z Offset. Don't ask why, I don't know)
    SHINGLE_HEIGHT = 0.1829
    SHINGLE_WIDTH  = 0.254
    OVERHANG = 0.0871
    HORIZ_OFFSET = 0.005
    Z_OFFSET = 0.12
    VERT_OFFSET = 0.019478

    if self.shingles_moved >= self.shingle_count:
      rospy.logerr("All available shingles moved, cannot spawn any more")

    # Odd row
    if roof_coord[1] % 2 == 1:
      x_pos = SHINGLE_WIDTH/2 + (SHINGLE_WIDTH + HORIZ_OFFSET)*(roof_coord[0])
    # Even row
    else:
      x_pos = (SHINGLE_WIDTH + HORIZ_OFFSET) * (roof_coord[0] + 1)

    y_pos = (SHINGLE_HEIGHT - OVERHANG + VERT_OFFSET) * (roof_coord[1])
    z_pos = Z_OFFSET + math.tan(math.radians(9.02)) * (SHINGLE_HEIGHT - OVERHANG + VERT_OFFSET) * (roof_coord[1])

    rx = math.radians(90)
    ry = math.radians(9.02)
    rz = -math.radians(90)

    quat = quaternion_from_euler(rx, ry, rz)

    model_name = f"inchworm::shingle_description_{self.shingles_moved}"

    req = SetModelStateRequest()
    
    req.model_state.model_name = model_name

    req.model_state.pose.position.x = x_pos
    req.model_state.pose.position.y = y_pos
    req.model_state.pose.position.z = z_pos

    req.model_state.pose.orientation.x = quat[0]
    req.model_state.pose.orientation.y = quat[1]
    req.model_state.pose.orientation.z = quat[2]
    req.model_state.pose.orientation.w = quat[3]

    req.model_state.reference_frame = "world"

    self.gazebo_move_service(req)
    self.gazebo_move_service(req)
    self.gazebo_move_service(req)
    self.gazebo_move_service(req)

    self.shingles_moved += 1

def pause():
  proxy = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

  proxy(EmptyRequest())

def unpause():
  proxy = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)

  proxy(EmptyRequest())

if __name__ == "__main__":
  # Shingle manager test

  rospy.init_node("shingle_manager_test")

  width = rospy.get_param("/roof_width")
  height = rospy.get_param("/roof_height")

  manager = ShingleManager(width, height)

  # manager.spawnShingle((0, 1))
  # manager.spawnShingle((0, 2))
  # manager.spawnShingle((0, 3))
  # manager.spawnShingle((0, 4))

  for col in range(width-1, -1, -1):
    manager.spawnShingle((col, 1))