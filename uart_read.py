import wiringpi2 as wiringpi
import time

wiringpi.wiringPiSetup()
serial = wiringpi.serialOpen('/dev/ttyS0',9600)
if serial == -1:
	print 'error opening serial at /dev/ttyS0'
	sys.exit(0)
print 'serial opened'
i = 0
while True:
	if (wiringpi.serialDataAvail(serial) > 0):
		line = wiringpi.serialGetChar(serial)
		print line
