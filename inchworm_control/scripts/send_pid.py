#!/usr/bin/env python
# license removed for brevity
import rospy
from inchworm_hw_interface.msg import PID, PIDConsts

def talker():
    pid_pub = rospy.Publisher("update_pid", PIDConsts, queue_size=1)
    p = 0.0
    i = 0.0
    d = 0.0
    pidF = PID(p, i, d)
    pidB = PID(p, i, d)
    pidConst = PIDConsts(pidF, pidB)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        input("Waiting for input")
        
        pid_pub.publish(pidConst)
        rate.sleep()
 
if __name__ == '__main__':
        talker()