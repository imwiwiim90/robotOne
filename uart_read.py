import wiringpi2 as wiringpi
import time

wiringpi.wiringPiSetup()
serial = wiringpi.serialOpen('/dev/ttyS0',9600)

i = 0
while True:
	if (wiringpi.serialDataAvail(serial)):
		line = wiringpi.serialGetChar(serial)
		print line
