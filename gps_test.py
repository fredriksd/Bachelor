#! usr/bin/env python

import time 
import serial

ser = serial.Serial(
	port = '/dev/serial0',
	baudrate = 9600,
	timeout = 15
	)


while True:
	data_in = ser.readline()
	if data_in[2:] == "RMC":
		print data_in


