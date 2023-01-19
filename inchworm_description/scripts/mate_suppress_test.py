#!/usr/bin/env python3

from contextlib import suppress
import rospy

from inchworm_hw_interface.msg import MagnetState
from assembly_msgs.srv import SuppressMate, SuppressMateRequest

from std_msgs.msg import Empty

suppress_proxy = None

def inchwormMagCB(msg):

  req = SuppressMateRequest()
  req.scoped_male = ["inchworm", "inchworm_description_0", "iw_foot_top_0"]
  req.scoped_female = ["inchworm", "shingle_description_0", "shingle_0"]
  req.suppress = True

  suppress_proxy(req)

def main():
  global suppress_proxy
  '''
  This node reads in state from the inchworm, and controls magnet suppression in the magnet sim.
  TODO: This script assumes one inchworm. Update to get robot count from param server and iterate over all.
  '''

  rospy.init_node("mate_suppress_test")
  
  rospy.loginfo("Waiting for /suppress_mate service...")
  rospy.wait_for_service("/suppress_mate")
  rospy.loginfo("Found service.")
  suppress_proxy = rospy.ServiceProxy("/suppress_mate", SuppressMate)

  rospy.Subscriber("/test", Empty, inchwormMagCB)

  rospy.spin()

if __name__ == "__main__":
  main()