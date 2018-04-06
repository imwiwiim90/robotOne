import socket
import sys
import time
import random

CHUNK_SIZE = 4096
try: 
    sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print 'Failed to create socket'
    sys.exit()

host = '198.211.112.16'
port = 8001

sckt.bind(('',0))
sckt.sendto('hello',(host,port))

msg, addr = sckt.recvfrom(CHUNK_SIZE)

print "addr {0} msg {1}".format(addr,msg)

host,port = msg.split(':')

sckt.settimeout(5.0)
while True:
	'''
	print 'sending to  {0}'.format(host + ':' + port)
	sckt.sendto(('hello from peer'),(host,int(port)))
	print 'receiving'
	try:
		msg, addr = sckt.recvfrom(CHUNK_SIZE)
		print msg
	except:
		pass
	'''
	print 'sending to  {0}'.format(host + ':' + port)
	sckt.sendto(('hello from peer'),(host,int(port)))
	print 'receiving'
	msg, addr = sckt.recvfrom(CHUNK_SIZE)
	print msg
	time.sleep(int(random.random()*5))
