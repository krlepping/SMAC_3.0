#!/usr/bin/env python3

import subprocess
import serial
import math

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

from matplotlib import pyplot as plt

ser = None

# List of 5 element arrays representing joint states at a timestamp (corresponds with the index in timestamps)
expected_joint_angles = []
actual_joint_angles = []

timestamps = []

debug_pub = None

def jointStateCB(msg):
    global ser, expected_joint_angles, actual_joint_angles

    joints = [math.degrees(j) for j in msg.position]
    expected_joint_angles.append(joints)

    serial_string = ""

    for i,j in enumerate(joints):
        to_add = ""

        to_add += "{:.2f}".format(j)

        # If its a positive joint value, we need to add a padding space
        if j >= 0:
            to_add = " " + to_add.zfill(6) + " "
        else:
            to_add = to_add.zfill(7) + " "

        print(f"Joint {i} string: {to_add}")

        serial_string += to_add

    serial_string += "0 0"

    print(f"Writing:\n\t{serial_string}")
    ser.write(serial_string.encode())

    read_string = ""
    read_string = str(ser.readline(), encoding="utf8").strip("\r\n")

    while read_string[0] == "E" and not rospy.is_shutdown():
        debug_pub.publish(String(read_string))
        read_string = str(ser.readline(), encoding="utf8").strip("\r\n")

    print(f"Read:\n\t{read_string}")

    joint_vals = [float(j) for j in read_string.split(" ")[:-2] if not j == ""]

    if not len(joint_vals) == 5:
        return

    actual_joint_angles.append(joint_vals)
    timestamps.append(rospy.Time.now())

def plotCB(_):
    expected_copy = expected_joint_angles[:]
    actual_copy = actual_joint_angles[:]
    ts_copy = timestamps[:]

    ts = processTimestamps(ts_copy)

    expected_1 = [j[0] for j in expected_copy]
    actual_1 =   [j[0] for j in actual_copy]

    print(len(ts))
    print(len(actual_1))
    print(len(expected_1))

    plt.plot(ts, expected_1, color="red", label="Expected")
    plt.plot(ts, actual_1, color="blue", label="Actual")

    plt.legend()

    plt.title("Expected vs Actual joint values")

    plt.show()

def processTimestamps(timestamps):
    out = [0]
    
    for i,t in enumerate(timestamps[1:]):
        out.append((t - timestamps[0]).to_sec())

    return out

def plot():
    # 5 element array, where each element is a single joint. Indexes into these sub arrays correspond with timestamps array
    joints = []

    # For each joint (or only for a specific subset of joints):
        # Convert into flat list of positions



    # Convert timestamps into floats that start at 0
    # Plot x,y data
    # Legend
    # Axis labels and title
    # Plot

    pass

if __name__ == "__main__":
    rospy.init_node("serial_interface")

    out = subprocess.run(["ls", "/dev"], capture_output=True)

    devices = str(out.stdout, encoding="utf8").split("\n")

    acm_devices = [d for d in devices if "ACM" in d]
    
    device = ""

    if len(acm_devices) == 1:
        device = acm_devices[0]
    else:
        print("Valid devices:")
        print("\n".join(acm_devices))

        device = acm_devices[int(input("Which device would you like? Please type the index from the list (0-indexed): "))]

    print(device)

    ser = serial.Serial("/dev/" + device, timeout=1)

    joint_sub = rospy.Subscriber("/inchworm/joint_states", JointState, jointStateCB, queue_size=1)
    plot_sub = rospy.Subscriber("/plot", Empty, plotCB, queue_size=1)

    debug_pub = rospy.Publisher("/debug", String, queue_size=1)

    rospy.spin()

    # while not rospy.is_shutdown():
    #     line = str(ser.readline(), encoding="utf8").strip("\r\n")

    #     print(line)