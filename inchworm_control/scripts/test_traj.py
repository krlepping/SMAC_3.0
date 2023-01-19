#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

start_pose = None

def poseCB(msg):
    global start_pose

    start_pose = msg

if __name__ == "__main__":
    rospy.init_node("test_traj")

    goal_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)
    state_sub = rospy.Subscriber("/inchworm/joint_goal", JointState, poseCB)

    while not rospy.is_shutdown() and start_pose is None:
        print(start_pose)
        rospy.sleep(0.5)

    start_pose_copy = start_pose

    target_positions = []

    for i in range(5):
        target_positions.append(float(input(f"Joint {i} target pose (rads): ")))
    total_time  = float(input("Total time (s): "))

    timestep = 0.05

    print(f"Motion time: {total_time}s")
    print(f"Timestep: {timestep*1000}ms")

    interp_pts = []

    for i in range(5):
        interp_pts.append(np.linspace(start_pose_copy.position[i], target_positions[i], int(total_time/timestep)))

    start = rospy.Time.now()

    input("Press enter to start trajectory")

    # Converts an array of the form [[joint 1 positions], [joint 2 positions], ..., [joint 5 positions]] to
    #                               [[j1, j2, j3, j4, j5 timestep 1], [j1, j2, j3, j4, j5 timestep 2], ...]
    # so that it can be put easily into JointTrajectoryPoints
    joint_states = list(zip(*interp_pts))

    traj = JointTrajectory()
    traj.header.frame_id = "world"
    traj.joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

    for i,state in enumerate(joint_states):
        pt = JointTrajectoryPoint()
        pt.positions = state
        pt.velocities = [0]*5
        pt.accelerations = [0]*5
        pt.effort = [0]*5

        pt.time_from_start = rospy.Duration(i * timestep)

        traj.points.append(pt)

    traj.header.stamp = rospy.Time.now()

    goal_pub.publish(traj)