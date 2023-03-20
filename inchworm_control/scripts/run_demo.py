#!/usr/bin/env python3

import rospy, glob, rospkg, yaml, sys

from inchworm_hw_interface.msg import MagnetState

from traj_planner import TrajectoryPlanner

mag_state_pub    = None

### TODO: This block should probably move to a library
def magnet(action):
  state = MagnetState()

  state.magnet1 = action["payload"]["magnet1"]
  state.magnet2 = action["payload"]["magnet2"]

  mag_state_pub.publish(state)
  rospy.sleep(float(action["duration"]))

def nfc(_):
  print("nfc")

def comm(_):
  print("comm")

def delay(action):
  rospy.sleep(float(action["duration"]))

######################################################

def runDemo(data, actions, planner):
  ACTION_FN_MAP = {"magnet": magnet, "nfc": nfc, "comm": comm, "delay": delay}

  # Run each action sequentially
  for action in actions:
    print(data)
    data = list(action.values())[0]

    print(action)
    print()

    name = list(action.keys())[0]

    # If the type is in our function map, invoke the function and pass in the action data
    if data["type"] in ACTION_FN_MAP:
      print(f"Running action {name}")
      ACTION_FN_MAP[data["type"]](data)
    elif data["type"] == "joint":
      print(f"Running action {name}")
      planner.run_quintic_traj(data["payload"], data["duration"])
    else:
      rospy.logerr(f"Invalid action type {data['type']}, quitting")
      sys.exit()

def main():
  global trajectory_pub, mag_state_pub
  rospy.init_node("demo_runner")

  mag_state_pub = rospy.Publisher("/inchworm/set_magnet_state", MagnetState, queue_size=1)

  rospack = rospkg.RosPack()

  demos = glob.glob(rospack.get_path("inchworm_control") + "/demos/*.yaml")

  planner = TrajectoryPlanner()

  print(f"Available demos: {' '.join([name.split('/')[-1][:-5] for name in demos])}")
  demo = input("Which demo would you like? Choose from the list above: ")

  path = f"{rospack.get_path('inchworm_control')}/demos/{demo}.yaml"

  data = None

  with open(path, "r") as f:
    try:
      data = yaml.safe_load(f)
    except yaml.YAMLError as e:
      print(e)
      sys.exit()

  actions = list(data.values())[0]

  if "loop" in demo:
    while not rospy.is_shutdown():
      runDemo(data, actions, planner)
      input()
  else:
    runDemo(data, actions, planner)

if __name__ == "__main__":
  main()