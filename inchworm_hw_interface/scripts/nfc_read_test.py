#!/usr/bin/env python3

import rospy, sys

from inchworm_hw_interface.srv import ReadNFCBlock, ReadNFCBlockRequest

def main():
  rospy.init_node("nfc_read_test")

  proxy = rospy.ServiceProxy("/inchworm/nfc_read", ReadNFCBlock)

  req = ReadNFCBlockRequest()

  req.bottom_foot = True
  req.block = 4

  res = proxy(req)

  block_data = res.data

  out = ""

  for byte in block_data:
    if byte >= 32 and byte <= 126:
      out += chr(byte)
    else:
      out += "."
  print(out)

if __name__ == "__main__":
  main()