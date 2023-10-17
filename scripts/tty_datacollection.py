import csv
import serial

test_name = input("Test Name: \n")
file_name = test_name + ".csv"

ser = serial.Serial('/dev/tty.usbserial-A50285BI', 19200)