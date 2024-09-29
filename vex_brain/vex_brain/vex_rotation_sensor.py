########################################################
#                                                      #
#Autor: BLAYES Hugo                                    #
#Created: 8/7                                          #
#Version: 1.0.0                                        #
#Description: ROS2 Node for communicate with           #
# Rotation sensor                                      #
########################################################
import rclpy
from rclpy.node import Node

import os
import serial
import threading
import sys
import argparse

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand

from std_msgs.msg import Int32

class Rotationsensor(Node):
	def __init__(self,port):
		super().__init__("rotationsensor_"+str(port))
	
		self.port = port 		
		
		self.publisher_ = self.create_publisher(Vexrotationsensor, 'out_rotationsensor_'+str(port), 10)
		self.in_publisher = self.create_publisher(Vexcommand, 'in_serial', 10)
		
		self.create_subscription(
			Int32,
			'setPosition_rotation_sensor_'+str(port),
			self.setPosition_callback,
			10	
		)
		
		self.subscription = self.create_subscription(
			Vexmsg,
			'out_serial',
			self.listener_callback,
			10
		)
		self.subscription
		
		
		
	def listener_callback(self,msg):
		rotation_sensor = msg.rotationsensor	
		
		rotation_out = Vexrotationsensor()
		
		for i in rotation_sensor:
			if i.port == self.port:
				rotation_out.port = i.port
				rotation_out.angle = i.angle
				rotation_out.velocity = i.velocity
		self.publisher_.publish(rotation_out)
		
	def setPosition_callback(self,msg):
		command = Vexcommand()
		
		command.object = "1"
		command.port = str(self.port)
		command.command = "1"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)	
	
def main(args=None):
	rclpy.init(args=args)
	
	temp_node = rclpy.create_node("temp")
	
	port = int(temp_node.declare_parameter("port",-1).get_parameter_value().integer_value)
	
	temp_node.destroy_node()
	
	rot = Rotationsensor(2)
	
	rclpy.spin(rot)
		
if __name__ == "__main__":
	main()
