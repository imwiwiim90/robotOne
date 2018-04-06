import time
import RPi.GPIO as GPIO

GPIO.setup(4,GPIO.OUT)
GPIO.setup(17,GPIO.OUT)


time.sleep(1)
GPIO.output(4,0)
GPIO.output(17,1)

time.sleep(1)

GPIO.output(17,0)
GPIO.output(4,1)

time.sleep(1)

GPIO.output(17,0)
GPIO.output(4,0)