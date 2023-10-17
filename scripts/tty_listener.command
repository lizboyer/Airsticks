#!/usr/bin/env python3
import serial

ser = serial.Serial('/dev/tty.usbserial-A50285BI', 19200)

while(True):
    data = ser.readline()
    if data:
        print(data)

