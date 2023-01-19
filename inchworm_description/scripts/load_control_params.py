#!/usr/bin/env python3

import rospy, sys, yaml, rospkg


def replaceN(params, n):
  '''
  Replaces all N characters in a nested dict with str(n) passed in
  '''
  replaced_params = {}

  for k,v in params.items():
    # If the value is a list, assume elements are strings and replace N
    if isinstance(v, list):
      v = [s.replace("N", str(n)) for s in v]
    # If the value is a dict, we need to recurse
    if isinstance(v, dict):
      replaced_params[k.replace("N", str(n))] = replaceN(v, n)
    # Otherwise, we can just replace the key name
    else:
      replaced_params[k.replace("N", str(n))] = v

  return replaced_params

def main(count):
  rospy.logwarn("I got here")

  rospack = rospkg.RosPack()
  base_path = rospack.get_path("inchworm_description")

  params = {}

  with open(base_path + "/config/inchworm_control.yaml") as f:
    params = yaml.safe_load(f)

  for i in range(count):
    p = replaceN(params, i)

    current_params = rospy.get_param(f"inchworm_{i}", {})

    # Merges two dictionaries
    p = {**p[f"inchworm_{i}"], **current_params}

    rospy.set_param(f"inchworm_{i}", p)

  rospy.logwarn("Done loading control parameters.")

if __name__ == "__main__":
  main(1)