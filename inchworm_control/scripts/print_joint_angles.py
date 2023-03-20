import numpy as np
import rospy, actionlib

from sensor_msgs.msg import JointState

class Joints:

    def __init__(self):
        self.current_joint_state = None
        joint_topic = f"/inchworm/joint_states"
        self.joint_sub = rospy.Subscriber(joint_topic, JointState, self.jointCB)


    def jointCB(self, msg):
        self.current_joint_state = msg
        

joints = Joints()
