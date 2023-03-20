#!/usr/bin/env python3

import rospy, rospkg, math, sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from inchworm_hw_interface.msg import MagnetState

time_to_disable = 5
time_between_traj_points = 5 # do not change

def toggleMagnet(mag_num, ee_publisher):
    ee_msg = MagnetState()
    if (mag_num == 2):
        ee_msg.magnet1 = True
        ee_msg.magnet2 = False
    else:
        ee_msg.magnet1 = False
        ee_msg.magnet2 = True

    ee_publisher.publish(ee_msg)
    rospy.sleep(time_to_disable)
    ee_msg.magnet1 = True
    ee_msg.magnet2 = True
    ee_publisher.publish(ee_msg)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        traj_file = sys.argv[1]
    else:
        traj_file = "step.csv"
    print(f"running the trajectory stored in {traj_file}")
    rospy.init_node("run_trajectory")

    joints_pub = rospy.Publisher("/inchworm/position_trajectory_controller/command", JointTrajectory, queue_size=1)

    ee_pub = rospy.Publisher("/inchworm/set_magnet_state", MagnetState, queue_size=1)


    rospack = rospkg.RosPack()

    traj_path = rospack.get_path("inchworm_control") + "/trajectories/" + traj_file

    print(traj_path)

    joints_msg = JointTrajectory()
    joints_msg.header.frame_id = "world"

    joints_msg.joint_names = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

    traj_pts = []

    with open(traj_path) as traj:
        traj_pts = [[math.radians(float(angle)) for angle in pt.split(",")] for pt in traj.readlines()]

    print(traj_pts)


    for i,pt in enumerate(traj_pts):

        joints_msg.points.append(JointTrajectoryPoint())

        joints_msg.points[-1].positions = [pt[0], pt[1], pt[2], pt[3], pt[4]]
        joints_msg.points[-1].velocities = [0] * time_between_traj_points
        joints_msg.points[-1].accelerations = [0] * time_between_traj_points

        joints_msg.points[-1].time_from_start = rospy.Duration((i + 1) * time_between_traj_points)
        if pt[5] > 0:
            pt[5] = 1
        if pt[6] > 0:
            pt[6] = 1
 
    print(joints_msg)
    # input()
    rospy.sleep(1)

    joints_msg.header.stamp = rospy.Time.now()
    joints_pub.publish(joints_msg)
    
    step_flag = False

    for i,pt in enumerate(traj_pts):
        mag_state = MagnetState(magnet1=pt[5], magnet2=pt[6])

        if pt[5] == 1:
            if step_flag:
                rospy.sleep(time_between_traj_points - time_to_disable)
            print("magnet 1 off")
            toggleMagnet(1, ee_pub)
            print("magnet 1 on")
            if step_flag:
                rospy.sleep(time_between_traj_points - time_to_disable)
            step_flag = not step_flag
        elif pt[6] == 1:
            if step_flag:
                rospy.sleep(time_between_traj_points - time_to_disable)
            print("magnet 2 off")
            toggleMagnet(2, ee_pub)
            print("magnet 2 on")
            if not step_flag:
                rospy.sleep(time_between_traj_points - time_to_disable)
            step_flag = not step_flag
        else:
            rospy.sleep(time_between_traj_points)
        print(f"step flag is: {step_flag}")
        
