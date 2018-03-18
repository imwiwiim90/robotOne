import wiringpi2 as wiringpi
import time

wiringpi.wiringPiSetup()
serial = wiringpi.serialOpen('/dev/ttyS0',9600)

i = 0
while True:
	wiringpi.serialPuts(serial,str(i%2))
	time.sleep(0.1)
	i+=1
