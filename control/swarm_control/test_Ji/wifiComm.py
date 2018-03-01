#!/usr/bin/python

# =====================================================================
# wifiComm.py
# =====================================================================
# last updated: August 11, 2017
# author: Martin Herrera mh866
#
# Script use to connect computers on the same network to send and recieve sphero commands
# Clients connect to the network and recieve velocity command packets from the server. Clients
# run as individual instances of the program and the server is initiated within the control script
# for the spheros as an object. 

import sys
import socket
import struct
import numpy as np

import sphero_nonRos

class wifiComm(object):
	# -------------------------
	# Instantiate object fields
	# -------------------------
	# bcastPort is arbitrary but must be the same between server and client
	# bcastIP is set to default as ""
	# Bcast is the Bcast address given by ifconfig
	bcastPort = 12345
	bcastIP = ""
	Bcast = '128.84.189.255'

	command_packet = []
	msg_fmt = ""
	msg_size = []

	def __init__(self):
		# -------------------------------------------------------------------------------
		# Initiate the format and size of the command packet and set it to array of zeros
		# -------------------------------------------------------------------------------
		self.msg_fmt = "3s1c3f"
		self.msg_size = struct.calcsize(self.msg_fmt)

	def _init_server (self):
		# -----------------------------------------------------
		# Initialize server object, set to broadcast on network
		# -----------------------------------------------------
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

	def sendCmdPacket(self,sphero,msg_type,data):
		# ---------------------------------------------------------
		# Pack command packet as binary data and broadcast
		# ---------------------------------------------------------
		# Determine whether velocity command or color command
		if (msg_type == 'v'):
			data = [data[0],data[1],0.0]
		msg = struct.pack(self.msg_fmt,sphero,msg_type,*data)
		self.sock.sendto(msg,(self.Bcast,self.bcastPort))

	def _init_client(self):
		# ------------------------------------------
		# Init client object and listen for commands
		# ------------------------------------------
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		# Bind to default broadcast IP to recieve messages
		self.sock.bind((self.bcastIP,self.bcastPort))
		print("Bound to socket")

		while True:
			# Listen to commands from Bcast, unpack commands from binary, pipe to appropriate spheros
			try:
				msg,addr = self.sock.recvfrom(self.msg_size)
				m = struct.unpack(self.msg_fmt,msg)
				print(m)
				colorCode = m[0]
				msg_type = m[1]
				if (colorCode in self.pipes):
					if(msg_type == 'v'):
						self.pipVelCmd(colorCode,m[2],m[3])
					else:
						self.pipeColCmd(colorCode,m[2],m[3],m[4])
			except KeyboardInterrupt:
				break

	def addSpheros(self,spheros):
		# -----------------------------------------------------------
		# Add spheros to list of relevant spheros for piping commands
		# -----------------------------------------------------------
		self.__init__()
		pipes = {}
		# Use sphero_nonRos to connect to appropriate sphero pipes to write to and 
		# add open file objects to array for writing
		for s in spheros:
			pipes[s]=sphero_nonRos.connectToSpheroPipe("Sphero_"+s)
		self.spheros = spheros
		self.pipes = pipes

	def pipVelCmd(self,target,vx,vy):
		# -----------------------------------------
		# Send command velocity to sphero via pipe 
		# -----------------------------------------
		print([vx,vy])
		sphero_nonRos.changeVel(self.pipes[target],[vx,vy])

	def pipeColCmd(self,target,R,G,B):
		# -----------------------------------------
		# Send color command to sphero via pipe 
		# -----------------------------------------
		sphero_nonRos.changeCol(self.pipes[target],[R,G,B])

if __name__ == '__main__':
	# -------------------------------------------------------------------
	# Create comm object and initiate it as a listener with given spheros
	# -------------------------------------------------------------------
	ob = wifiComm()
	spheros = sys.argv[1:]
	ob.addSpheros(spheros)

	print("Initializing Client")
	ob._init_client()

