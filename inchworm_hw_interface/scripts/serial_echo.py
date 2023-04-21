#!/usr/bin/env python3

import serial

ser = serial.Serial("/dev/ttyACM0", 9600, timeout=2)

try:
    while True:
        data = ser.read(1).__repr__()
        if data:
            print("Received: %s." % data)
        else:
            print("Looping.")
except KeyboardInterrupt:
    print("Done.")
except:
    raise
finally:
    ser.close()
    print("Closed port.")
