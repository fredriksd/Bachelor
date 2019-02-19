#-*- coding:utf-8-*-

import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b00
spi.cshigh = False


#Legge den i en egen fil? Litt "ryddigere"
#Begrenser val mellom min_ og max_
def constrain(val, min_, max_):
	return max(min(max_,val),min_)


class PWM():
	'''PWM-klasse for motorutgangene. PWM-utgangen er PD-regulert
	og kan inverteres etter behov.
	'''

	def __init__(self,initial_position = 1500, P_gain = 400, D_gain = 300, inverted = False):
		self.position = initial_position
		self.inverted = inverted
		self.firstupdate = True
		self.P_gain = P_gain
		self.D_gain = D_gain
		self.previous_error = 0

	def update(self, error):
		if self.inverted:
			error *= -1 
		if self.firstupdate == False:
			error_delta = error - self.previous_error
			vel = (self.P_gain * error + error_delta * self.D_gain)/1024

			self.position += vel
			#Begrenser utslagene på servoutgangen mellom 1000us
			#og 2000us.
			self.position = constrain(self.position, 1000, 2000)
			#print self.position
			return self.position
			#print self.position
		else:
			self.firstupdate = False
		self.previous_error = error


def get_Pixy():
	send = []
	i = 0
	resp = spi.xfer([0xaa55,0xaa55])
	
	while resp[0] == 0 and resp[1] == 0 or not resp:
		resp = spi.xfer([0xaa55,0xaa55])
		return False
	if resp[0] == 170 and resp[1] == 85:
		while i < 7:
			resp = spi.xfer([0xaa55,0xaa55])
			#a = resp[0]
			#b = resp[1]
			if i == 3: #x-posisjon
				send.append(resp[1])
			elif i == 4: #y-posisjon
				send.append(resp[1])
			elif i == 5: #bredde
				send.append(resp[1])
			elif i == 6: #høyde
				send.append(resp[1])
			#send.append(a)
			#send.append(b)
			time.sleep(0.01)
			i += 1
	return send



