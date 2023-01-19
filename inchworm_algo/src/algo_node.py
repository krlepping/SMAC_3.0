#!/usr/bin/env python3

import rospy, sys, random, time

from roof import Roof
from inchworm import Inchworm, EEStatus
from std_msgs.msg import Int32, Bool, Int32MultiArray
from shingle import ShingleStatus
from inchworm_algo.msg import RoofState, InchwormsMsg
from inchworm_algo.srv import GetInchwormState, GetInchwormStateResponse
from std_srvs.srv import Empty, EmptyRequest


# Globals that can be accessed by ROS callbacks
r = None
inchworms = []
paused = False
ticks = 0

def rateCB(msg):
    global r
    r = rospy.Rate(msg.data)

def pauseCB(msg):
    global paused
    paused = msg.data
    

def handle_get_inchworm_state(req):
    idx = 0
    for i, worm in enumerate(inchworms):
        if worm.id == req.inchworm_idx:
            idx = i
    return GetInchwormStateResponse(state=inchworms[idx].to_message())

def spawn_inchworms(roof, inchworm_count, pat, use_physics):
        inchworm_count = min(int(roof.width/2), inchworm_count)
        inchworms = []
        for inchworm_id in range(inchworm_count):
            inchworms.append(Inchworm(id=inchworm_id, bottom_foot_pos=[inchworm_id * 2, 0], top_foot_pos=[(inchworm_id*2) + 1, 0], width=roof.width, height=roof.height, top_foot_stat=EEStatus.PLANTED, pattern=pat, physics=use_physics))
            roof.inchworm_occ[0][inchworm_id * 2] = 1
            roof.inchworm_occ[0][(inchworm_id*2) + 1] = 1
        return inchworms


def update_inchworms(roof, inchworms):
    global ticks
    ticks += 1
    is_done = False
    random.shuffle(inchworms)
    if ticks % 1000 == 0:
        rospy.loginfo(ticks)
    for worm in inchworms:
        if worm is not None:
            worm.make_decision(roof)
    for worm in inchworms:
        end_shingle = roof.get_shingle(0, roof.height - 1)
        if end_shingle is not None and end_shingle.shingle_status == ShingleStatus.INSTALLED:
            for row in roof.shingle_array:
                for shingle in row:
                    if shingle.shingle_status != ShingleStatus.INSTALLED:
                        rospy.logerr("roof shingling failed!!!!")
                        rospy.signal_shutdown("roof failed to be shingled")
            is_done = True
        if worm is not None and not is_done:
            worm.run_action(roof)
    return is_done, roof, inchworms

def create_inchworms_msg(inchworms):
    inchworm_msg = InchwormsMsg()
    for inchworm in inchworms:
        inchworm_msg.inchworms.append(inchworm.to_message())
    inchworm_msg.header.stamp = rospy.Time.now()
    return inchworm_msg

if __name__ == "__main__":
    rospy.init_node("algo_node")

    roof_width = int(sys.argv[1])
    roof_height = int(sys.argv[2])

    hz = 5
    inchworm_count = int(sys.argv[3])
    if len(sys.argv) >= 4:
        hz = int(sys.argv[4])
    use_physics = sys.argv[6] == "True"
    rospy.logwarn(use_physics)
    roof = Roof(roof_width, roof_height, False, use_physics)

    pattern = int(sys.argv[5])
    
    inchworms = spawn_inchworms(roof, inchworm_count, pattern, use_physics)

    roof_pub = rospy.Publisher("algo/roof_state", RoofState, queue_size=1)
    algo_finished_pub = rospy.Publisher("/algo/ticks_elapsed", Int32MultiArray, queue_size=1)
    inchworm_pub = rospy.Publisher("algo/inchworms", InchwormsMsg, queue_size=1)

    rate_sub = rospy.Subscriber("algo/rate", Int32, rateCB)
    pause_sub = rospy.Subscriber("algo/pause", Bool, pauseCB)

    rospy.Service("algo/get_inchworm_state", GetInchwormState, handle_get_inchworm_state)

    r = rospy.Rate(hz)
    status = False
    rospy.sleep(2) # time it takes to startup the algo viz

    start_time = time.time()

    while not rospy.is_shutdown() and not status:
        if not paused:

            roof_msg = roof.to_message()
            for worm in inchworms:
                roof_msg.inchworms.append(worm.to_message())
            roof_pub.publish(roof_msg)
            status, roof, inchworms = update_inchworms(roof, inchworms)
            # inchworm_pub.publish(create_inchworms_msg(inchworms))
            if status:
                rospy.loginfo("roof has been shingled")
                finished_msg = Int32MultiArray()
                move_counts = []
                state_data = []
                for worm in inchworms:
                    move_counts.append(worm.move_count)
                    state_data.extend(list(worm.state_counts.values()))
                rospy.loginfo(state_data)
                finished_msg.data = [len(inchworms), ticks]
                finished_msg.data.extend(move_counts)
                finished_msg.data.extend(state_data)
                algo_finished_pub.publish(finished_msg)
        r.sleep()
    if use_physics:
        rospy.loginfo("ALGO SIM FINISHED")
        end_time = time.time()
        dt = end_time - start_time
        rospy.loginfo(f"Took {dt} seconds to run.")
        pause_physics_client=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        pause_physics_client(EmptyRequest())
