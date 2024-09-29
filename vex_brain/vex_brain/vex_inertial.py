########################################################
#                                                      #
#Autor: BLAYES Hugo                                    #
#Created: 8/7                                          #
#Version: 1.0.0                                        #
#Description: ROS2 Node for communicate with Vex Imu   #
#                                                      #
########################################################


import rclpy
from rclpy.node import Node

import os
import serial
import argparse
import sys

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand
from vex_message.msg import Vexinertial

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Empty

class Inertial(Node):
	def __init__(self,port):
		super().__init__("inertial_"+str(port))
		
		#port of the module on the vex 
		self.port = port

		self.publisher_ = self.create_publisher(Vexinertial, 'out_inertial_'+str(port), 10)
		self.in_publisher = self.create_publisher(Vexcommand, 'in_serial', 10)
		
		self.create_subscription(
			Int32,
			'StartCalibration_inertial_'+str(port),
			self.StartCalibration_inertial_callback,
			10
		)
		
		self.create_subscription(
			Empty,
			'Calibrate_inertial_'+str(port),
			self.Calibrate_inertial_callback,
			10
		)
		
		self.create_subscription(
			Empty,
			'Resetheading_inertial_'+str(port),
			self.Resetheading_inertial_callback,
			10
		)
		
		self.create_subscription(
			Float32,
			'Setheading_inertial_'+str(port),
			self.Setheading_inertial_callback,
			10
		)
		
		self.create_subscription(
			Float32,
			'SetRotation_inertial_'+str(port),
			self.SetRotation_inertial_callback,
			10
		)
		
		self.sub = self.create_subscription(
			Vexmsg,
			'out_serial',
			self.inertial_callback,
			10
		)
		self.sub
	
	#this function get out_serial message to transform into Vexinertial message 
	def inertial_callback(self,msg):
		inertial = msg.inertial	
		
		inertial_out = Vexinertial()
		
		for i in inertial:
			if i.port == self.port:
				inertial_out.port = i.port
				inertial_out.accelerationx = i.accelerationx
				inertial_out.accelerationy = i.accelerationy
				inertial_out.accelerationz = i.accelerationz
				inertial_out.heading = i.heading
				inertial_out.orientation_pitch = i.orientation_pitch
				inertial_out.orientation_roll = i.orientation_roll
				inertial_out.orientation_raw = i.orientation_raw
				inertial_out.rotation = i.rotation
				
		self.publisher_.publish(inertial_out)

	#we transform all the message in command
	def StartCalibration_inertial_callback(self,msg):
		command = Vexcommand()
		
		command.object = "3"
		command.port = str(self.port)
		command.command = "1"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def Calibrate_inertial_callback(self,msg):
		command = Vexcommand()
		
				
		command.object = "3"
		command.port = str(self.port)
		command.command = "2"
		
		self.in_publisher.publish(command)
		
	def Resetheading_inertial_callback(self,msg):
		command = Vexcommand()
		
				
		command.object = "3"
		command.port = str(self.port)
		command.command = "3"
		
		self.in_publisher.publish(command)
		
	def Setheading_inertial_callback(self,msg):
		command = Vexcommand()
		
				
		command.object = "3"
		command.port = str(self.port)
		command.command = "4"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def SetRotation_inertial_callback(self,msg):
		command = Vexcommand()
		
				
		command.object = "3"
		command.port = str(self.port)
		command.command = "5"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		

def main(args=None):
	rclpy.init(args=args)
	
	#we create a temp node to get the parameter port to create the node 
	temp_node = rclpy.create_node("temp")
	
	port = int(temp_node.declare_parameter("port",-1).get_parameter_value().integer_value)
	
	#and destroy it after 
	temp_node.destroy_node()
	
	inertial = Inertial(port)
	
	rclpy.spin(inertial)
	
if __name__ == "__main__":
	main()
