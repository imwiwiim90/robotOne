import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

GPIO.setup(4,GPIO.OUT)
GPIO.setup(17,GPIO.OUT)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)


def left():
	GPIO.output(4,0)
	GPIO.output(17,1)

def right():
	GPIO.output(17,0)
	GPIO.output(4,1)

def center():
	GPIO.output(17,0)
	GPIO.output(4,0)

def forward():
	GPIO.output(6,0)
	GPIO.output(5,1)

def backward():
	GPIO.output(5,0)
	GPIO.output(6,1)

def stop():
	GPIO.output(5,0)
	GPIO.output(6,0)

while True:
	stop()
	k = raw_input('left (l), right (r), center(c)')
	if k == 'l':
		left()
	if k == 'r':
		right()
	if k == 'c':
		center()
	if k == 'q':
		break
	k = raw_input('forward (f), backward (b)')
	if k == 'f':
		forward()
	if k == 'b':
		backward()
	if k == 'q': 
		break
	k = raw_input('press anything to stop')

