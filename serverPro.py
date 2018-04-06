import socket
import sys
import time

CHUNK_SIZE = 4096
PORT = 8001

try: 
    sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print 'Failed to create socket'
    sys.exit()


try:
    sckt.bind(("", PORT)) # as server
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
print 'Socket bind complete'

driver = {
	'addr': None,
	'time': 0,
}
robot = {
	'addr': None,
	'time': 0,
}

TIMEOUT = 15

while True:
	msg, addr = sckt.recvfrom(CHUNK_SIZE)
	print '{0} {1}'.format(addr,msg)
	entity,action = msg.split('|')

	if time.time() - driver['time'] > TIMEOUT:
		driver['addr'] = None
	if time.time() - robot['time'] > TIMEOUT:
		robot['addr'] = None

	if entity == 'DRIVER':
		driver['addr']  = addr
		driver['time'] = time.time()
		other_addr = robot['addr']
	elif entity == 'ROBOT':
		robot['addr'] = addr
		robot['time'] = time.time()
		other_addr = driver['addr']

	if action == 'PEER':
		msg = 'null'
		if not other_addr == None:
			msg = other_addr[0] + ':' + str(other_addr[1])
		sckt.sendto(msg,addr)


