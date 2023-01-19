#!/usr/bin/env python3

import serial, rospy, subprocess, struct, inspect, math

from threading import Lock

from std_msgs.msg import Int32, String
from sensor_msgs.msg import JointState

from inchworm_hw_interface.msg import MagnetState, PIDConsts, PID

# Baud rate to communicate with the Teensy
BAUD = 9600

# If True, then the serial port is not connected. All writes are simply logged
DEBUG = False

VERBOSE = True

# Publishers for each kind of message embedded software can send to us
heartbeat_pub = None
joint_poses_pub = None
joint_goal_pub = None
pid_consts_pub = None
magnet_state_pub = None
debug_pub = None
fault_pub = None

# Subscribers for each kind of message we can send to embedded software
heartbeat_sub = None
joint_goal_sub = None
pid_consts_sub = None
magnet_state_sub = None

serial_port = None
ser_mutex = Lock()

def send_serial(to_send):
    # Embedded software expects all messages to be newline terminated
    to_send += b"\n"

    if DEBUG:
        suppress = ["send_joint_goal"]

        if not inspect.stack()[1].function in suppress:
            print(f"Function {inspect.stack()[1].function} sending:")
            print(to_send)
            print()
    else:
        if VERBOSE:
            print(f"Sending {chr(to_send[0])}")
            #print(to_send)

        ser_mutex.acquire()
        serial_port.write(to_send)
        ser_mutex.release()

def heartbeat(byte_arr):
    print("Receiving heartbeat")
    print(byte_arr)
    seq = struct.unpack("<ixxxx", byte_arr)[0]
    print(seq)

    heartbeat_msg = Int32(seq)

    heartbeat_pub.publish(heartbeat_msg)

def joint_poses(byte_arr):
    # Unpack 10 contiguous doubles
    poses = struct.unpack("<" + "d"*15, byte_arr)

    if True:
        debug_str = String(f"Pose arr: {poses}")
        debug_pub.publish(debug_str)

    joint_state_msg = JointState()

    joint_state_msg.header.stamp = rospy.Time.now()

    # TODO: Figure out the naming convention for joints, because this is bad and inconsistent
    joint_state_msg.name = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

    for pos in poses[:5]:
        joint_state_msg.position.append(math.radians(pos))

    for vel in poses[5:10]:
        joint_state_msg.velocity.append(vel)

    for effort in poses[10:]:
        joint_state_msg.effort.append(effort)

    joint_poses_pub.publish(joint_state_msg)

def joint_goal(byte_arr):
    # Unpack 10 contiguous doubles
    poses = struct.unpack("<" + "d"*5, byte_arr)

    joint_state_msg = JointState()

    joint_state_msg.header.stamp = rospy.Time.now()

    # TODO: Figure out the naming convention for joints, because this is bad and inconsistent
    joint_state_msg.name = ["iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"]

    for pos in poses[:5]:
        joint_state_msg.position.append(math.radians(pos))

    joint_goal_pub.publish(joint_state_msg)

def pid_consts(byte_arr):
    consts = struct.unpack("<" + "d"*30, byte_arr)

    tuples = list((consts[3*i], consts[3*i+1], consts[3*i+2]) for i in range(int(len(consts)/3)))

    consts_msg = PIDConsts()

    for i in range(5):
        forward = PID()
        backward = PID()

        forward.p = tuples[i][0]
        forward.i = tuples[i][1]
        forward.d = tuples[i][2]

        backward.p = tuples[i+5][0]
        backward.i = tuples[i+5][1]
        backward.d = tuples[i+5][2]

        consts_msg.forward.append(forward)
        consts_msg.backward.append(backward)

    pid_consts_pub.publish(consts_msg)

def magnet_state(byte_arr):
    print(byte_arr)
    mag_states = struct.unpack("<ii", byte_arr)

    mag_state_msg = MagnetState()

    mag_state_msg.magnet1 = mag_states[0]
    mag_state_msg.magnet2 = mag_states[1]

    magnet_state_pub.publish(mag_state_msg)

def debug(byte_arr):
    message = struct.unpack(">100s", byte_arr)
    message = message[0][:message[0].index(b"\n")]

    try:
        debug_msg = String(message.decode("utf-8"))
    except UnicodeDecodeError:
        fault_msg = String("Bad message parsed while reading debug")
        fault_pub.publish(fault_msg)
        return

    debug_pub.publish(debug_msg)

def fault(byte_arr):
    message = struct.unpack(">100s", byte_arr)
    message = message[0][:message[0].index(b"\n")]

    try:
        fault_msg = String(message.decode("utf-8"))
    except UnicodeDecodeError:
        fault_msg = String("Bad message parsed while reading fault")

    fault_pub.publish(fault_msg)

def send_heartbeat(msg):
    command = struct.pack(">cxxxxxxx", b"h")
    data = struct.pack("<ixxxx", msg.data)

    to_send = command + data

    print(to_send)

    send_serial(to_send)

def send_joint_goal(msg):
    command = struct.pack(">cxxxxxxx", b"g")
    data = struct.pack("<" + "d"*5, *[math.degrees(p) for p in msg.position])

    to_send = command + data

    send_serial(to_send)

def send_pid_consts(msg):
    consts_arr = []

    f = msg.forward
    b = msg.backward

    if not len(f) == len(b):
        rospy.logerr(f"send_pid_consts: Mismatch on joint count. f: {f} b: {b}")

    # len(f) === len(b) by now, so we can just check one
    if len(f) != 5:
        rospy.logerr(f"send_pid_consts: Got {len(f)} joints, needed 5")
        return

    for pid in f:
        consts_arr.extend([pid.p, pid.i, pid.d])
    for pid in b:
        consts_arr.extend([pid.p, pid.i, pid.d])

    command = struct.pack(">cxxxxxxx", b"p")
    data = struct.pack("<" + "d"*30, *consts_arr)

    to_send = command + data

    print(to_send)
    print(len(to_send))

    send_serial(to_send)

def send_magnet_states(msg):
    command = struct.pack(">cxxxxxxx", b"m")
    data = struct.pack("<ii", msg.magnet1, msg.magnet2)

    to_send = command + data

    print(to_send)

    send_serial(to_send)

def init_pubs():
    global heartbeat_pub, joint_poses_pub, joint_goal_pub, pid_consts_pub, magnet_state_pub, debug_pub, fault_pub

    heartbeat_pub = rospy.Publisher("heartbeat_res", Int32, queue_size=1)
    joint_poses_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    joint_goal_pub = rospy.Publisher("joint_goal", JointState, queue_size=1)
    pid_consts_pub = rospy.Publisher("pid_consts", PIDConsts, latch=True, queue_size=1)
    magnet_state_pub = rospy.Publisher("magnet_states", MagnetState, queue_size=1)
    debug_pub = rospy.Publisher("debug", String, queue_size=1)
    fault_pub = rospy.Publisher("fault", String, queue_size=1)

def init_subs():
    global heartbeat_sub, joint_goal_sub, pid_consts_sub, magnet_state_sub

    heartbeat_sub = rospy.Subscriber("heartbeat_req", Int32, send_heartbeat, queue_size=5)
    joint_goal_sub = rospy.Subscriber("joint_state_test", JointState, send_joint_goal, queue_size=1)
    pid_consts_sub = rospy.Subscriber("update_pid", PIDConsts, send_pid_consts, queue_size=5)
    magnet_state_sub = rospy.Subscriber("set_magnet_states", MagnetState, send_magnet_states, queue_size=5)

def init_serial():
    global DEBUG

    out = subprocess.run(["ls", "/dev"], capture_output=True)

    devices = str(out.stdout, encoding="utf8").split("\n")

    acm_devices = [d for d in devices if "ACM" in d]
    
    device = ""

    # If there are no devices matching the pattern, assume we should be in debug mode
    if len(acm_devices) == 0:
        print("No ACM device found, enabling debug mode.")
        DEBUG = True
        return None

    elif len(acm_devices) == 1:
        device = acm_devices[0]

    else:
        print("Valid devices:")
        print("\n".join(acm_devices))

        device = acm_devices[int(input("Which device would you like? Please type the index from the list (0-indexed): "))]

    print(device)

    ser = serial.Serial("/dev/" + device, BAUD, timeout=2)

    # Definitely secure handshake protocol. You don't say "hhhhh" when you're first greeting someone?
    handshake = b"hhhhh"
    ser.write(handshake)

    res = ser.read(5)

    if not res == handshake:
        print("Response not correct from robot.")
    else:
        print("Received startup message, communication established.")

    return ser

# Maps type characters from incoming serial messages to parser functions
# Key: Type character
# Value: (message_size, handler_fn)
# More info in https://docs.google.com/document/d/1m7oZZbM0VJFrIxo1KRNSXnIhmvcEmCzY7amgNRJE87Q/edit?usp=sharing
char_fn_map = {
    b"h": (16, heartbeat),
    # 8 command + sizeof(double[15])
    b"j": (128, joint_poses),
    # 8 command + sizeof(double[5])
    b"g": (48, joint_goal),
    b"p": (248, pid_consts),
    b"m": (16, magnet_state),
    b"d": (108, debug),
    b"f": (108, fault)
}

if __name__ == "__main__":
    rospy.init_node("message_parser")

    serial_port = init_serial()

    init_pubs()
    init_subs()

    while not rospy.is_shutdown():
        # Read in the type char if the serial port initialized
        if serial_port is not None:
            type_char = serial_port.read(1)
        else:
            rospy.sleep(1)
            continue

        # If it's a valid command, parse it. Otherwise, skip this character
        if type_char in char_fn_map:
            if VERBOSE:
                # Message command characters to ignore
                suppress = ["d", "j"]
                if type_char.decode() not in suppress:
                    print(f"Received {type_char.decode()} message")

            # Skip 7 padding bytes
            _ = serial_port.read(7)

            # Read in the specified number of bytes, minus 4 for type_char+3*padding bytes
            byte_arr = serial_port.read(char_fn_map[type_char][0] - 8)

            # if VERBOSE:
            #    print(byte_arr)

            # Pass the byte array to the appropriate handler
            char_fn_map[type_char][1](byte_arr)