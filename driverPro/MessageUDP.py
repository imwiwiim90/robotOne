import socket
import time
import sys


class MessageUDP(object):
	def __init__(self):
		self.server_addr = ('198.211.112.16',8001)
		#create socket
		try:
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		except socket.error:
			print 'Failed to create socket'
			sys.exit()

		# socket listen
		try:
			self.socket.bind(('',0)) # as server
		except socket.error , msg:
			print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
			sys.exit()
		print 'Socket bind complete'

		# server ping attrs
		self.last_server_time = 0
		self.server_ping_time = 5
		self.robot_addr = None
	def request_robot_addr(self):
		sckt = self.socket
		try:
			sckt.settimeout(3.0)
			sckt.sendto('DRIVER|PEER',self.server_addr)
			msg, addr = sckt.recvfrom(CHUNK_SIZE)
			if msg == 'null':
				self.robot_addr = None
				return
			host,port = msg.split(':')
			self.robot_addr = (host,int(port))
		except:
			self.robot_addr = None

	def ping_server(self):
		self.socket.sendto('DRIVER|PING',self.server_addr)

	def send(self,string):
		if self.robot_addr == None:
			self.request_robot_addr()
			self.last_server_time = time.time()
			time.sleep(3)
			return False

		if time.time() - self.last_server_time > self.server_ping_time:
			self.ping_server()
			self.last_server_time = time.time()

		# send msg
		try:
			self.socket.sendto(string, self.robot_addr)
		except:
			return False

		# receive msg
		try:
			msg,addr = self.socket.recvfrom(CHUNK_SIZE)
			return msg
		except:
			return False