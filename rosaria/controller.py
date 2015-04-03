#!/usr/bin/env python
import roslib
import rospy
import math
import numpy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import thread
import math
from commandListener import CommandListenerNode


class ControllerNode():
	def __init__(self):
		self.twist = Twist()		
		self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist)
		self.CurrentPosition = Vector3(0,0,0)
		self.DesiredPosition = Vector3(0,0,0)
		self.TargetOrientation = Vector3(0,0,0)
		self.CurrentOrientation = 0
		self.ManualMode = True
		self.emergency = False
		
		self.MaxSpeed = .3
		self.Speed = .3
		
		self.MaxTurningSpeed = .6
		self.TurningSpeed = .6
		
		#Allowed to turn and be off 5 degrees. Updates will go consistently
		#to make sure it stays within that range
		self.AngelTolerance = 5
		self.DistanceTolerance = .3
		self.ID = 0
		#self.X = 0
		#self.Y = 0
		#self.Z = 0
		self.Roll = 0
		self.Pitch = 0
		self.Yaw = 0
		self.idnode = 0
	
		#self.IDNode()
		self.PoseNode()
		self.AtDestination = False
	
	def IDNode(self):
		self.idnode = rospy.Subscriber('ID', String, self.HandleIDData)
		print "OdometryNode activated"
		
		

	def HandleIDData(self, msg):
		self.ID = msg.data
		#print("Pirning ID from callback" + self.ID)
		#self.idnode.shutdown()
		# print "HandlingOdometry"
	
	def setEmergencyOn(self):
		self.emergency = True
		
	def setEmergencyOff(self):
		self.emergency = False
	
	def PoseNode(self):
		rospy.Subscriber('/RosAria/pose', Odometry, self.HandlePoseData)
		#print "OdometryNode activated"
	
	def HandlePoseData(self, msg):
		Quat_angle = msg.pose.pose.orientation
		quaternion = []
		quaternion.append(Quat_angle.x)
		quaternion.append(Quat_angle.y)
		quaternion.append(-Quat_angle.z)
		quaternion.append(Quat_angle.w)
		
		eulerVal = euler_from_quaternion(quaternion)
		#Determine how many degrees have been rotated from starting orientation.
		self.Roll = eulerVal[0] * 180.0 / math.pi
		self.Pitch = eulerVal[1] * 180.0 / math.pi
		self.Yaw = eulerVal[2] * 180.0 / math.pi
		self.X = msg.pose.pose.position.x
		self.Y = msg.pose.pose.position.y
		self.Z = msg.pose.pose.position.z
		
		self.CurrentPosition = Vector3(self.X, self.Y, self.Z)
		self.CurrentOrientation = float(self.Yaw)
			
	def GoTo(self, DesiredPosition):
		self.ManualMode = False
		self.AtDestination = False
		self.DesiredPosition = DesiredPosition
		print("Command: GoTo")

	def Move(self, Direction):
		self.ManualMode = True
		self.twist = Twist()

		if self.emergency == False:
		#Move Forward
			if Direction == 2:
				self.twist.linear.x = self.Speed;
				pass
			#Turn Right
			elif Direction == 4:
				self.twist.angular.z = -self.TurningSpeed		
				pass
			#Move Backwards
			elif Direction == 6:
				self.twist.linear.x = -self.Speed;
				pass
			#Turn Left
			elif Direction == 8:
				self.twist.angular.z = self.TurningSpeed		
				pass
			
			
			#Move Forward while Turning Left
			elif Direction == 1:
				self.twist.linear.x = self.Speed;
				self.twist.angular.z = self.TurningSpeed
				pass
			#Move Forward while Turning Right
			elif Direction == 3:
				self.twist.linear.x = self.Speed;
				self.twist.angular.z = -self.TurningSpeed
			#Move Backwards towards right rear
			elif Direction == 5:
				self.twist.linear.x = -self.Speed;
				self.twist.angular.z = self.TurningSpeed
				pass
			#Move Backwards towards left rear
			elif Direction == 7:
				self.twist.linear.x = -self.Speed;
				self.twist.angular.z = -self.TurningSpeed		
				pass
			print("Command: Move in Direction: " + str(Direction))
			self.pub.publish(self.twist)
	
	def Stop(self):
		print("Command: Stop")
		self.ManualMode = True
		self.twist = Twist()
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.pub.publish(self.twist)
		rospy.sleep(.1)
	
	def Update(self):
		while True:
			if self.emergency == True:
				print("Emergency State")
				pass	#emergency must be reset
			else:
				if self.ManualMode == True:
					pass	#print("ManualMode")
				else: 
					self.AIUpdate()	
	
	def AIUpdate(self):
	
		#test first to see if kill variable is set to true
		#if emergency, stop
		if self.emergency:
			self.TargetOrientation = self.CurrentOrientation
			self.DesiredPosition = self.CurrentPosition
			self.AtDestination == True
			self.Stop()
		
		else:
			#Test to see if we are at our goal
			if self.AtDestination == False:
				self.twist = Twist()
		
		
				#print(self.DesiredPosition)
				self.TargetOrientation = math.atan2(self.DesiredPosition.y - self.CurrentPosition.y, self.DesiredPosition.x - self.CurrentPosition.x) * 180/math.pi
				print("TargetOrientation: " + str(self.TargetOrientation))
				print("CurrentOrientation: " + str(self.CurrentOrientation))			

				Turn = 0
				ClockWise = 0
			

						

				if (self.TargetOrientation > 0 and self.CurrentOrientation < 0) or  self.TargetOrientation < 0 and self.CurrentOrientation > 0:
					OrientationDifference = self.TargetOrientation + self.CurrentOrientation
				else:
					OrientationDifference = self.TargetOrientation - self.CurrentOrientation		
				Rotation = math.fabs(OrientationDifference)
				if Rotation > 180:
					Rotation = 360-Rotation
			


				print("OrientationDifference: "+str(OrientationDifference))
				if math.fabs(Rotation) < self.AngelTolerance:
					Turn = 0
					#Go straight
					self.twist.angular.z = 0
					self.twist.linear.x = self.MaxSpeed
					pass
				else:		
					if OrientationDifference > 0:
						ClockWise = 1
						pass
						#Go clockwise
					else:
						ClockWise = -1
						pass
						#Go counterclockwise

				
					if math.fabs(Rotation) < 30:
						self.twist.linear.x = .5
						self.twist.angular.z = ClockWise * self.MaxTurningSpeed
					#Just turn so we don't move forward in wrong direction				
					else:
						self.twist.linear.x = .0				
					
					self.twist.angular.z = ClockWise * self.MaxTurningSpeed
				
				#Go towards goal: DesiredPosition
			

				print("CurrentPos: " + str(self.CurrentPosition))
				print("Desired: " + str(self.DesiredPosition))
				DiffVec = Vector3(0, 0, 0)
				DiffVec.x = self.DesiredPosition.x - self.CurrentPosition.x
				DiffVec.y = self.DesiredPosition.y - self.CurrentPosition.y
				DistanceToGoal = math.sqrt(DiffVec.x**2 + DiffVec.y**2)
				print("Distance to goal is: " + str(DistanceToGoal))
				if DistanceToGoal < self.DistanceTolerance:
					self.AtDestination = True
					self.twist = Twist()
			
			else:
				self.twist.linear.x = 0
				self.twist.angular.z = 0	
				#print("AIMode")
			self.pub.publish(self.twist)
			rospy.sleep(.05)	

if __name__ == '__main__':
	rospy.init_node('controller')
	Controller = ControllerNode()
	print("starting controller")

	CmdListener = CommandListenerNode(Controller)
	CmdListener.Start()
	print("starting commandlistener")
	
	rospy.Rate(10)
	while True:
		Controller.Update()
		rospy.spinOnce()
