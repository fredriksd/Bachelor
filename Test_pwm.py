import RPi.GPIO as IO         

import time



IO.setwarnings(False)           

IO.setmode (IO.BCM)      

IO.setup(19,IO.OUT)



p = IO.PWM(19,100)        	 
p.start(50)

while True:
	pass


'''
while 1:

    for x in range (50):
        p.ChangeDutyCycle(x)
        time.sleep(1)
      
    for x in range (50):           
        p.ChangeDutyCycle(50-x) 
        time.sleep(1)                
'''
