#!/usr/bin/env python3

import rospy

from inchworm_algo.srv import GetInchwormState, GetInchwormStateRequest

def main():
    rospy.init_node("inspect_state")

    proxy = rospy.ServiceProxy("/algo/algo/get_inchworm_state", GetInchwormState)

    while not rospy.is_shutdown():
        idx = int(input("Inchworm index: "))

        req = GetInchwormStateRequest(inchworm_idx=idx)
        res = proxy(req)

        print(f"Inchworm {idx} state:")
        print(res)

if __name__ == "__main__":
    main()