#!/usr/bin/env python3

import rospy

from inchworm_hw_interface.msg import PIDConsts, PID

if __name__ == "__main__":
    rospy.init_node("pub_consts")

    pub = rospy.Publisher("update_pid", PIDConsts, queue_size=1)

    consts = PIDConsts()

    for i in range(5):
        f = PID(3*i, 3*i+1, 3*i+2)
        b = PID(3*i+15, 3*i+16, 3*i+17)

        consts.forward.append(f)
        consts.backward.append(b)

    input("About to publish, press enter")
    pub.publish(consts)
    input("Published")