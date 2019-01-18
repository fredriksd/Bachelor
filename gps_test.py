#! usr/bin/env python

import time 
import serial

ser = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 9600,
	timeout = 15
	)

while True:
	data_in = ser.readline()
	print data_in


