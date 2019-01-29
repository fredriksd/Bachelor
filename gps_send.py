import RPi.GPIO as GPIO
import time
import serial

ser = serial.Serial(
	port='/dev/serial0',
	baudrate=9600,
	timeout=15
	)

while True:
	#data_in = ser.readline()
	#print data_in
	ser.write("hello")
	time.sleep(1)

