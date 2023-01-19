#!/usr/bin/env python3

import rospy, random, time

from inchworm_hw_interface.srv import ReadNFCBlock, ReadNFCBlockRequest, WriteNFCBlock, WriteNFCBlockRequest

def main():
  rospy.init_node("nfc_rw_test")

  print("Waiting for services...")

  rospy.wait_for_service("/inchworm/nfc_write")
  rospy.wait_for_service("/inchworm/nfc_read")

  write_proxy = rospy.ServiceProxy("/inchworm/nfc_write", WriteNFCBlock)
  read_proxy = rospy.ServiceProxy("/inchworm/nfc_read", ReadNFCBlock)

  TEST_DATA = [random.randint(0, 255) for i in range(16)]

  print("Test data:")
  print(TEST_DATA)

  write_req = WriteNFCBlockRequest()
  write_req.bottom_foot = True
  write_req.block = 4
  write_req.data = TEST_DATA

  res = write_proxy(write_req)

  print("Made write request.")
  time.sleep(3)

  read_req = ReadNFCBlockRequest()

  read_req.bottom_foot = True
  read_req.block = 4

  res = read_proxy(read_req)

  block_data = [int(elem) for elem in res.data]

  print(block_data)

  if(block_data != TEST_DATA):
    print("Read did not match write :(")
  else:
    print("Read matched write!")

if __name__ == "__main__":
  main()