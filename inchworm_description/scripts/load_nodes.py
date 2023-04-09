#!/usr/bin/env python3

import rospy, sys, yaml, rospkg
import load_control_params, load_controller_spawners, load_robot_urdf, load_state_publishers


def main():
    rospy.init_node("node_loader")

    count = rospy.get_param("~robot_count", 1)

    # load two nodes
    load_control_params.main(count)
    load_robot_urdf.main(count)
    # wait for them to do the thing
    #until rosnode list | grep "node"; do
    #    pass
    # load two more nodes 
    load_controller_spawners.main(count)
    load_state_publishers.main(count)

if __name__ == "__main__":
  main()
