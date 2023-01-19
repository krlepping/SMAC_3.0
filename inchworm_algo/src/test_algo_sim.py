#!/usr/bin/env python3

import rospy, roslaunch, math
from rospkg import RosPack
from matplotlib import pyplot as plt
import sys
import csv

import numpy as np
from inchworm import Pattern

from std_msgs.msg import Int32MultiArray





durations = []
running = 0


def chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]


def tickCB(msg):
  global running
  global durations
  running -= 1
  durations.append(msg.data)

def main():
  global running
  global durations
  rospy.init_node("test_algo_sim")

  WIDTH = 10
  HEIGHT = 10
  RATE = 500



  pattern = int(sys.argv[1])
  WIDTH = int(sys.argv[2])
  HEIGHT = int(sys.argv[3])

  INCHWORM_COUNTS = [i+1 for i in range(int(WIDTH/2))]

  tick_sub = rospy.Subscriber("/algo/ticks_elapsed", Int32MultiArray, tickCB)

  uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
  roslaunch.configure_logging(uuid)
  
  rospack = RosPack()
  algo_path = rospack.get_path("inchworm_algo")

  launch_path = [algo_path + "/launch/algo_sim.launch"]


  batch_size = 9
  if int(int(WIDTH)/2) < batch_size:
    batch_size = int(int(WIDTH)/2)


  count = 1
  while len(durations) != int(int(WIDTH)/2) and not rospy.is_shutdown():

    if running < batch_size and count <= int(int(WIDTH)/2):
      print(f"Running test for inchworm count of {count}")

      cli_args = [f"roof_width:={WIDTH}", f"roof_height:={HEIGHT}", f"rate:={RATE}", f"pattern:={pattern}", f"use_gui:=False", f"inchworm_count:={count}", f"name_space:={WIDTH}x{HEIGHT}_{pattern}_{count}"]


      launch_file = [(roslaunch.rlutil.resolve_launch_arguments(launch_path + cli_args)[0], cli_args)]
      launch = roslaunch.parent.ROSLaunchParent(uuid, launch_file)

      launch.start()
      count += 1
      running += 1


    rospy.sleep(0.5)
    # rospy.loginfo(len(durations))

 

  durations.sort(key=lambda x: x[0])
  file = open(f"{algo_path}/data/{WIDTH}x{HEIGHT}_pattern{pattern}.csv", "w+")
  writer = csv.writer(file)
  writer.writerows(durations)
  x = []
  y = []
  for element in durations:
    x.append(element[0])
    y.append(element[1])
  durations = y.copy()
  INCHWORM_COUNTS = x.copy()
  plt.scatter(x, durations)

  plt.xlabel("Inchworm count")
  plt.ylabel("Total ticks elapsed")
  plt.title(f"Time to shingle a {WIDTH}x{HEIGHT} roof with pattern {pattern}")

  # ATTEMPTING A POWER REGRESSION TO GET A BEST FIT LINE
  log_of_inchworms = []
  log_of_duration = []
  duration_times_inchworms = []
  inchworms_squared = []
  durations_squared = []

  for i, inch in enumerate(INCHWORM_COUNTS):
    log_of_inchworms.append(math.log(inch)) # x
    log_of_duration.append(math.log(durations[i])) # y
    duration_times_inchworms.append(log_of_inchworms[i]*log_of_duration[i])
    inchworms_squared.append(log_of_inchworms[i]*log_of_inchworms[i])
    durations_squared.append(log_of_duration[i]*log_of_duration[i])

  n = len(INCHWORM_COUNTS)
  numerator = n*sum(duration_times_inchworms) - (sum(log_of_duration)*sum(log_of_inchworms))
  denominator = n*sum(inchworms_squared) - (sum(log_of_inchworms) * sum(log_of_inchworms))
  
  B = numerator/denominator

  avg_log_duration = sum(log_of_duration)/len(log_of_duration)
  avg_log_inchworm = sum(log_of_inchworms)/len(log_of_inchworms)

  A_not = avg_log_duration - B*(avg_log_inchworm)
  A = math.exp(A_not)

  print(f"best fit line is y = {round(A, 5)}x^{round(B, 5)}")

  # calculating the r^2 value using Pearsons correlation formula

  std_dev_duration = []
  std_dev_inchworm = []
  mutliplied_std = []
  for i, none in enumerate(log_of_inchworms):
    std_dev_duration.append(log_of_duration[i]- avg_log_duration)
    std_dev_inchworm.append(log_of_inchworms[i] - avg_log_inchworm)
    mutliplied_std.append(std_dev_duration[i]*std_dev_inchworm[i])

  x_denom = denominator
  y_denom = n*sum(durations_squared) - (sum(log_of_duration) * sum(log_of_duration))
  r = numerator/math.sqrt(x_denom*y_denom)

  r_squared = r**2
  print(f"r is {r} r^2 is equal to {r_squared}")

  # graphing the best fit curve 
  x = np.linspace(INCHWORM_COUNTS[0], INCHWORM_COUNTS[-1], 100)
  y = A*x**B

  plt.plot(x, y, label=f"y = {round(A, 3)}x^{round(B, 3)}, \n r^2 = {round(r_squared, 5)}", color='m')
  plt.legend()

  plt.yscale("log")
  # plt.xscale("log")
  # Uncomment if you want to add a limit, Need to know a good top value 
  # ax = plt.gca()
  # ax.set_ylim(1,45000)

  plt.savefig(f"{algo_path}/data/{WIDTH}x{HEIGHT}_{pattern}.png")

if __name__ == "__main__":
  main()