import socket
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
            self.socket.bind(("",0)) # as server
        except socket.error , msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()
        print 'Socket bind complete'

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

	def set_destination(self,ip,port):
		self.host = ip
		self.port = port


	def send(self,string):
		try:
			self.socket.sendto(string, (self.host,self.port))
		except:
			print "connection unsuccesful"