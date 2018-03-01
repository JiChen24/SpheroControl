#!/usr/bin/python
import sys
import socket
import struct
import numpy as np
import rospy
import geometry_msgs.msg

class spheroComm(object):
	packet_indices =   {'WPP': 0,
						'WPW': 2,
						'RPP': 4,
						'RPR': 6,
						'GGW': 8,
						'OBR': 10,
						'PRR': 12}
	bcastPort = 12345
	bcastIP = ""
	mask = '128.84.189.255'

	command_packet = []
	msg_fmt = ""
	msg_size = []

	def __init__(self):
		self.command_packet = np.zeros(len(self.packet_indices)*2)
		self.msg_fmt = str(len(self.packet_indices)*2)+"f"
		self.msg_size = struct.calcsize(self.msg_fmt)

	def getIndSphero(self,target):
		try:
			return self.packet_indices[target]
		except KeyError:
			print("Key error, sphero code not defined")
			return -1

	def _init_server (self):

		rospy.init_node('Sphero_Comm_Server')

		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

	def add_vel_cmd(self,target,vel):
		ind = self.getIndSphero(target)
		self.command_packet[ind] = vel[0]
		self.command_packet[ind+1] = vel[1]

	def sendCmdPacket(self):
		msg = struct.pack(self.msg_fmt,*self.command_packet)
		self.sock.sendto(msg,(self.mask,self.bcastPort))

	def _init_client(self):
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.sock.bind((self.bcastIP,self.bcastPort))

		rospy.init_node('Sphero_Comm_Client')

		while True:
			try:
				msg,addr = self.sock.recvfrom(self.msg_size)
				m = struct.unpack(self.msg_fmt,msg)
				for sphero_name in self.spheros:
					self.pipVelCmd(m,sphero_name)
			except KeyboardInterrupt:
				break

	def addSpheros(self,spheros):
		sphero_pubs = {}
		for s in spheros:
			target_name = "sphero_"+s.lower()
			sphero_pubs[s] = rospy.Publisher(target_name+"/cmd_vel",geometry_msgs.msg.Twist, queue_size=1, latch=True)
		self.spheros = sphero_pubs

	def parseVelCmd(self,msg,target):
		ind = self.getIndSphero(target)
		try:
			return [msg[ind],msg[ind+1]]
		except KeyError:
			return -1

	def pipVelCmd(self,msg,target):
		cmd = self.parseVelCmd(msg,target)
		vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(cmd[0],cmd[1],0), \
                                              geometry_msgs.msg.Vector3(0,0,0))
		print(self.spheros)
		self.spheros[target].publish(vel_msg)

if __name__ == '__main__':
	#Interpret argument as server or client
	ob = spheroComm()
	if sys.argv[1] == '1':
		print("init server")
		s = ob._init_server()
		ob.add_vel_cmd('RPP',[70.0, 0])
		ob.add_vel_cmd('PRR',[0.0, 0])
		print("sent")
		ob.sendCmdPacket()
	else:
		spheros = sys.argv[1:]
		ob.addSpheros(spheros)
		print("init client")
		ob._init_client()

