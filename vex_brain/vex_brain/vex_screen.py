#########################################################
#                                                       #
#Autor: BLAYES Hugo                                     #
#Created: 8/7                                           #
#Version: 1.0.0                                         #
#Description: ROS2 Node for communicate with Vex Screen #
#                                                       #
#########################################################

import rclpy
from rclpy.node import Node

import os
import serial
import threading

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand

from vex_message.msg import Drawcircle
from vex_message.msg import Drawline
from vex_message.msg import Point
from vex_message.msg import Color
		
from std_msgs.msg import Int32
from std_msgs.msg import Empty
from std_msgs.msg import String	
		
class Screen(Node):
	def __init__(self):
		super().__init__("screen")
		
		self.port = 0
		
		self.publisher_ = self.create_publisher(Vexscreen, 'out_screen', 10)
		self.in_publisher = self.create_publisher(Vexcommand, 'in_serial', 10)
		
		self.create_subscription(
			Int32,
			"clearLine_screen",
			self.clearLine_callback,
			10
		)
		
		self.create_subscription(
			Empty,
			"clearScreen_screen",
			self.clearScreen_callback,
			10
		)
		
		self.create_subscription(
			Drawcircle,
			"drawCircle_screen",
			self.drawCircle_callback,
			10
		)
		
		self.create_subscription(
			Drawline,
			"drawLine_screen",
			self.drawLine_callback,
			10
		)
		
		self.create_subscription(
			Drawline,
			"drawRect_screen",
			self.drawRect_callback,
			10
		)
		
		self.create_subscription(
			Empty,
			"newLine_Screen",
			self.newLine_callback,
			10
		)
		
		self.create_subscription(
			String,
			"print_Screen",
			self.printScreen_callback,
			10
		)
		
		self.create_subscription(
			Point,
			"SetCursor_Screen",
			self.setCursor_callback,
			10
		)
		
		self.create_subscription(
			Color,
			"setFillColor_Screen",
			self.setFillColor_callback,
			10
		)
		
		self.create_subscription(
			Color,
			"setPenColor_Screen",
			self.setPenColor_callback,
			10
		)
		
		self.create_subscription(
			Int32,
			"setPenWidth_Screen",
			self.setPenWidth_callback,
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
		screen = msg.screen
		
		screen_out = Vexscreen()
		
		screen_out.positionx = screen.positionx
		screen_out.positiony = screen.positiony
		screen_out.pressing = screen.pressing
		
		self.publisher_.publish(screen_out)
		
	def clearLine_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "1"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)	
		
	def clearScreen_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "2"
		
		self.in_publisher.publish(command)	
		
	def drawCircle_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "3"
		command.valeur = str(msg.x) + "-" + str(msg.y) + "-" + str(msg.radius)
		
		self.in_publisher.publish(command)	
		
	def drawLine_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "4"
		command.valeur = str(msg.x) + "-" + str(msg.y) + "-" + str(msg.x2) + "-" + str(msg.y2)
		
		self.in_publisher.publish(command)	
		
	def drawRect_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "5"
		command.valeur = str(msg.x) + "-" + str(msg.y) + "-" + str(msg.x2) + "-" + str(msg.y2)
		
		self.in_publisher.publish(command)	
		
	def newLine_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "6"
		
		self.in_publisher.publish(command)	
		
	def printScreen_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "7"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)	
		
	def setCursor_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "8"
		command.valeur = str(msg.x) + "-" + str(msg.y)
		
		self.in_publisher.publish(command)	
	
	def setFillColor_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "9"
		command.valeur = str(msg.r) + "-" + str(msg.g) + "-" + str(msg.b)
		
		self.in_publisher.publish(command)	
	
	def setPenColor_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "A"
		command.valeur = str(msg.r) + "-" + str(msg.g) + "-" + str(msg.b)
		
		self.in_publisher.publish(command)	
		
	def setPenWidth_callback(self,msg):
		command = Vexcommand()
		
		command.object = "0"
		command.port = "0"
		command.command = "B"
		command.valeur = str(msg.data)
		
		self.in_publisher.publish(command)	
		
def main(args=None):
	rclpy.init(args=args)
	
	screen = Screen()
	
	rclpy.spin(screen)	
	
if __name__ == "__main__":
	main()
