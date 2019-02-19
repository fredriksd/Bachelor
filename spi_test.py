import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 2200000
spi.mode = 0b00
spi.cshigh = False

while True:
	resp = spi.readbytes(16)
	#resp = spi.xfer([0xaa55])
	#string = "".join(map(chr, resp))
	time.sleep(0.05)
	#print string
	print resp
