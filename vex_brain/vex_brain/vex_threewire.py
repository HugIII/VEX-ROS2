###########################################################
#                                                         #
#Autor: BLAYES Hugo                                       #
#Created: 8/7                                             #
#Version: 1.0.0                                           #
#Description: ROS2 Node for communicate with Vex 3 wire   #
#                                                         #
###########################################################

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
from vex_message.msg import Vexthreewire

from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Empty

class Threewire(Node):
	def __init__(self,port):
		super().__init__("threewire_"+str(port))
		
		self.port = port
		self.publisher_ = self.create_publisher(Vexthreewire, 'out_threewire_'+str(port), 10)
		
		self.sub = self.create_subscription(
			Vexmsg,
			'out_serial',
			self.threewire_callback,
			10
		)
		self.sub
		
	def threewire_callback(self,msg):
		threewire = msg.threewire	
		
		threewire_out = Vexthreewire()
		
		for i in threewire:
			if i.port == self.port:
				threewire_out.port = i.port
				threewire_out.pressing = i.pressing
				
		self.publisher_.publish(threewire_out)
		
		

def main(args=None):
	rclpy.init(args=args)
	
	temp_node = rclpy.create_node("temp")
	
	port = temp_node.declare_parameter("port","-1").get_parameter_value().string_value
	
	temp_node.destroy_node()
	
	threewire = Threewire(port)
	
	rclpy.spin(threewire)
	
if __name__ == "__main__":
	main()
