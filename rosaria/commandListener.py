import socket
import thread
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class CommandListenerNode():
	def __init__(self, Controller):
		self.sock = 0
		self.run = True
		self.tokens = 0;
		self.Controller = Controller
		self.ID = self.Controller.ID
	
	def CreateIPSocket(self):
		#Internet, UDP
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.bind(('',8052))
		self.sock.settimeout(1)
		
	def ReceiveCommand(self):
		while self.run:
			try:
				data = self.sock.recv(1024)
			except socket.timeout:
				print("Too long since last command\nStopping")
				self.ProcessStopCommand()
			print("Received msg:" + data)
		#Parse through command
			self.tokens = data.split()
			print(self.tokens)
			
			#All commands have command type and unit ID
			if len(self.tokens) > 2:
				#Make sure message is command type which is 2
				if self.tokens[0] == "2":
					#Make sure message is being sent to correct robot
					#if self.tokens[1] == self.ID:
					if True:
						#Determine which command type is being sent
						if self.tokens[2] == "0":
							self.ProcessGoToCommand()
						elif self.tokens[2] == "1":
							self.ProcessStopCommand()
						elif self.tokens[2] == "2":
							pass
						elif self.tokens[2] == "3":
							pass
						elif self.tokens[2] == "4":
							pass
						elif self.tokens[2] == "5":
							self.ProcessMoveCommand()
						elif self.tokens[2] == "6":
							self.EmergencyOnCommand()
						elif self.tokens[2] == "7":
							self.EmergencyOffCommand()
						else:
							print("Invalid Command Type Received")
					else:
						print("Invalid ID")
			else:
				print("Command had invalid arguments")
						
	def ProcessGoToCommand(self):
		if len(self.tokens) == 6:
			try:
				#Flip Y and Z since coordinate systems are different for Ros and Unity3d
				DesiredPosition = Vector3(float(self.tokens[3]), float(self.tokens[5]), float(self.tokens[4]))
				#DesiredPosition = "DesPos"
				print("Desired Position: " + str(DesiredPosition))
				self.Controller.GoTo(DesiredPosition)
			except:
				print("Invalid arguments for x, y, z used")
			pass
		else:
			print("Invalid # Arguments: Expected 6: Received " + len(self.tokens))
			
	def ProcessStopCommand(self):
		try:
			self.Controller.Stop()
		except:
			print("Stop failed")
			
	def EmergencyOnCommand(self):
		try:
			self.Controller.setEmergencyOn()
			print("****Emergency state initiated****")
		except:
			print("****Emergency state ERROR****")
			
	def EmergencyOffCommand(self):
		try:
			self.Controller.setEmergencyOff()
			print("****Reset Emergency state*****")
		except:
			print("****Reset Emergency ERROR*****")
	
	def ProcessMoveCommand(self):
		if len(self.tokens) == 4:
			try:
				Direction = int(self.tokens[3])
				print("Movement Direction: " + str(Direction))
				self.Controller.Move(Direction)
			except:
				print("Move failed")
			pass
		else:
			print("Invalid # Arguments: Expected 6: Received " + len(self.tokens))
			
	def Start(self):
		self.CreateIPSocket()		
		thread.start_new_thread(self.ReceiveCommand, ())
