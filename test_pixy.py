import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)


while True:
	spi.xfer([0xff])
	resp = spi.readbytes(1)
	if resp!="0":
		print resp
	else:
		pass 

	time.sleep(0.5)

