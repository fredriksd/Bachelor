import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1)
spi.max_speed_hz = 2200000
spi.mode = 0b01
spi.cshigh = True

while True:
	resp = spi.readbytes(10)
	#resp = spi.xfer2([0xff,0xff]) 
	time.sleep(1)
	print resp
