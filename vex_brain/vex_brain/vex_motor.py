########################################################
#                                                      #
#Autor: BLAYES Hugo                                    #
#Created: 8/7                                          #
#Version: 1.0.0                                        #
#Description: ROS2 Node for communicate with Vex Motor #
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

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Empty

class Motor(Node):
	def __init__(self,port):
		super().__init__("motor_"+str(port))
		
		self.port = port
		self.publisher_ = self.create_publisher(Vexmotor, 'out_motor_'+str(port), 10)
		self.in_publisher = self.create_publisher(Vexcommand, 'in_serial', 10)
		
		self.create_subscription(
			Int32,
			'setMaxTorque_motor_'+str(port),
			self.motor_setMaxTorque_callback,
			10
		)
		
		self.create_subscription(
			Int32,
			'setPosition_motor_'+str(port),
			self.motor_setPosition_callback,
			10
		)
		
		self.create_subscription(
			Int32,
			'setTimeout_motor_'+str(port),
			self.motor_setTimeout_callback,
			10
		)
		
		self.create_subscription(
			Int32,
			'setVelocity_motor_'+str(port),
			self.motor_setVelocity_callback,
			10
		)
		
		self.create_subscription(
			Int32,
			'setSpin_motor_'+str(port),
			self.motor_setSpin_callback,
			10
		)
		
		self.create_subscription(
			Empty,
			'Stop_motor_'+str(port),
			self.motor_Stop_callback,
			10
		)
		
		self.create_subscription(
			Float32,
			'SpinVolt_motor_'+str(port),
			self.motor_SpinVolt,
			10
		)
		
		self.sub = self.create_subscription(
			Vexmsg,
			'out_serial',
			self.motor_callback,
			10
		)
		self.sub
		
	def motor_callback(self,msg):
		mot = msg.motor	
		
		mot_out = Vexmotor()
		
		for i in mot:
			if i.port == self.port:
				mot_out.port = i.port
				mot_out.isspining = i.isspining
				mot_out.isdone = i.isdone
				
		self.publisher_.publish(mot_out)

	def motor_setMaxTorque_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "1"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def motor_setPosition_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "2"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
	
	def motor_setTimeout_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "3"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def motor_setVelocity_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "4"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def motor_setSpin_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "5"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)
		
	def motor_Stop_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "6"
		
		self.in_publisher.publish(command)
		
	def motor_SpinVolt(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "7"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command) 
		
		

def main(args=None):
	rclpy.init(args=args)
	
	temp_node = rclpy.create_node("temp")
	
	port = int(temp_node.declare_parameter("port",-1).get_parameter_value().integer_value)
	
	temp_node.destroy_node()
	
	motor = Motor(port)
	
	rclpy.spin(motor)
	
if __name__ == "__main__":
	main()
