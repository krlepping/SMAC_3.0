#!/usr/bin/env python3

import rospy, roslaunch, math
from rospkg import RosPack
from matplotlib import pyplot as plt
import sys
import csv

import numpy as np


from std_msgs.msg import Int32MultiArray

WIDTH = 25
HEIGHT = 25
pattern = 1

def main():
    raw_data = []
    file = open(f"25x25_pattern1.csv", "r")
    spamreader = csv.reader(file, delimiter=' ', quotechar='|')
    for row in spamreader:
        data = row[0].split(',')
        x = int(data[0])
        print(data)
        y = [int(i) for i in data[1:len(data)]]
        y = max(y)
        y *= 16
        y /= 3600
        y /= 24
        raw_data.append([x, y])
        
    x = []
    y = []
    for element in raw_data:
        x.append(element[0])
        y.append(element[1])
    durations = y.copy()
    INCHWORM_COUNTS = x.copy()
    plt.scatter(x, durations)

    plt.xlabel("Inchworm count")
    plt.ylabel("Total estimated days")
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

    # plt.plot(x, y, label=f"y = {round(A, 3)}x^{round(B, 3)}, \n r^2 = {round(r_squared, 5)}", color='m')
    # plt.legend()

    # plt.yscale("log")
    # plt.xscale("log")
    # Uncomment if you want to add a limit, Need to know a good top value 
    # ax = plt.gca()
    # ax.set_ylim(1,45000)
    plt.show()
    plt.savefig(f"{WIDTH}x{HEIGHT}_{pattern}.png")


if __name__=='__main__':
    main()