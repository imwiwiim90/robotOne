from controller_alt import PS3Controller
import time
import json

ctrl = PS3Controller()

while True:
	print json.dumps(ctrl.getKeys(), indent=2)
	time.sleep(1)