import socket
import sys


class MessageUDP(object):
	def __init__(self):
		try:
			self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			self.socket.settimeout(1.0)
		except socket.error:
		    print 'Failed to create socket'
		    sys.exit()


	def set_destination(self,ip,port):
		self.host = ip
		self.port = port


	def send(self,string):
		try:
			self.socket.sendto(string, (self.host,self.port))
			print 'sent'
			msg, addr = self.socket.recvfrom(4096)
			return msg
		except:
			print "connection unsuccesful"