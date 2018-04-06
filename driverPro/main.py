from controller_alt import PS3Controller
from MessageUDP import MessageUDP
import json
import threading
import time

lock = threading.Lock()

#ip_dir = '192.168.0.5'
#ip_dir = '127.0.0.1'
#ip_dir = '186.31.47.239'
#ip_dir = '192.168.0.109'
#ip_dir = '190.24.134.186'
ip_dir = '186.155.238.2'
PORT = 1575
#PORT = '8030'
ctrl = PS3Controller()
mailer = MessageUDP()

#mailer.set_destination(ip_dir,PORT)

while True:
	time.sleep(1/30.0)
	message = ctrl.getKeys()
	print json.dumps(message, indent=2)
	msg = mailer.send(json.dumps(message))
	if msg:
		print msg

