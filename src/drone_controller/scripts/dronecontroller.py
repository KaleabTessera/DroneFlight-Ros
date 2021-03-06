#!/usr/bin/env python
# Note this has been adapted from https://github.com/mikehamer/ardrone_tutorials_getting_started
import roslib; 
import rospy

from  gazebo_msgs.srv import GetModelState
# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# An enumeration of Drone Statuses
from DroneStatus import DroneStatus

import tf
from math import  cos,sin
import argparse


class BasicDroneController:
	def __init__(self):
		rospy.init_node('BasicDroneController')
		# Holds the current drone status
		self.status = -1

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty,queue_size=10)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=10)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty,queue_size=10)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
		self.odom_broadcaster = tf.TransformBroadcaster()
		# Setup regular publishing of control packets

		self.command = Twist()
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,x,y,z):
		# Called by the main program to set the current command
		self.command.linear.x  = x
		self.command.linear.y  = y
		self.command.linear.z  = z

	def SendCommand(self):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)
	
	# Basic Up and Down Flight
	def run(self):
		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			self.SendTakeoff()
			rospy.sleep(1)
			self.SendLand()
			rate.sleep()
	
	def navigate(self,PID,dest):
		rate = rospy.Rate(10)
		PID.setInitialTime(time.time())
		while not rospy.is_shutdown():
			self.SendTakeoff()
			rospy.sleep(1)
			
			O_x, O_y, O_z,shouldStop = PID.apply(drone.getGazeboState(),dest)
			
			if(shouldStop):
				break

			self.SetCommand(O_x,O_y,O_z)
			self.SendCommand()
			rate.sleep()
			
	def getGazeboState(self):
		model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		quad_coordinates = model_coordinates('quadrotor','')
		z = quad_coordinates.pose.position.z
		y = quad_coordinates.pose.position.y
		x = quad_coordinates.pose.position.x
		# print(quad_coordinates)
		return quad_coordinates
		

from nav_msgs.msg import Odometry
import time
class PIDController:
	def __init__(self,iniState):
		self.iniState = iniState
		self.K_I = 0.01
		self.K_D = 0.5
		self.K_P = 0.6
		self.error_history = 0

		self.errorThreshold = 0.05
		self.last_error_x =0 
		self.last_error_y =0 
		self.last_error_z =0 

		self.integral_x = 0
		self.integral_y = 0
		self.integral_z = 0

	def setInitialTime(self,t=0):
		self.last_time = t
	
	def apply(self,currentState,finalState):
		shouldStop = None	
		z_c = currentState.pose.position.z
		y_c = currentState.pose.position.y
		x_c = currentState.pose.position.x

		z_f = finalState[2]
		y_f = finalState[1]
		x_f = finalState[0]

		e_t_x = x_f - x_c
		e_t_y = y_f - y_c
		e_t_z = z_f - z_c

		P_x = self.K_P * e_t_x 
		P_y = self.K_P * e_t_y 
		P_z = self.K_P * e_t_z 

		self.current_time = time.time()

		delta_time = self.current_time - self.last_time

		
		self.integral_x += e_t_x * delta_time
		self.integral_y += e_t_y * delta_time	
		self.integral_z += e_t_z * delta_time

		

		I_x = self.K_I * self.integral_x
		I_y = self.K_I * self.integral_y
		I_z = self.K_I * self.integral_z

		der_x = (e_t_x - self.last_error_x) /delta_time
		der_y = (e_t_y - self.last_error_y) /delta_time
		der_z = (e_t_z - self.last_error_z) /delta_time

		D_x = self.K_D * der_x
		D_y = self.K_D * der_y
		D_z = self.K_D * der_z
		
		self.last_time = self.current_time
		self.last_error_x = e_t_x
		self.last_error_y = e_t_y
		self.last_error_z = e_t_z

		O_x = P_x + I_x + D_x
		O_y = P_y + I_y + D_y
		O_z = P_z + I_z + D_z
		
		print("Error: x:{} y:{} z:{}".format(e_t_x,e_t_y,e_t_z))
		print("Current: {} Dest: {}".format([x_c,y_c,z_c],[x_f,y_f,z_f]))

		if((abs(e_t_x) < self.errorThreshold * x_f) and (abs(e_t_y) < self.errorThreshold * y_f) and (abs(e_t_z) < self.errorThreshold * z_f)):
			print("Stopping, error threshold {} : Final Coords: [{},{},{}]".format(self.errorThreshold,x_c,y_c,z_c))			
			shouldStop = True 

		return O_x,O_y,O_z,shouldStop
		
if __name__ == '__main__':
		parser = argparse.ArgumentParser(description='PID Controller.')
		parser.add_argument('--dest', '--list', help='delimited list input', type=str)
		args = parser.parse_args()
		
		x = 3
		y = 2
		z = 5

		if(args.dest):
			args.dest = args.dest.split(',')
			x = int(args.dest[0])
			y = int(args.dest[1])
			z = int(args.dest[2])
		try:
			drone = BasicDroneController()
			pid_con = PIDController(drone.getGazeboState())
			drone.navigate(pid_con,[x,y,z])
		except:
    			pass


