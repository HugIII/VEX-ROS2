####################################################
#                                                  #
#Autor: BLAYES Hugo                                #
#Created: 8/7                                      #
#Version: 1.0.0                                    #
#Description: ROS2 Node for talk with a vex        #
#                                                  #
####################################################

#include
import rclpy
from rclpy.node import Node

import os
import serial
from threading import Lock
import time

#include vex message
from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand
from vex_message.msg import Vexinertial
from vex_message.msg import Vexthreewire

class Talker(Node):
	
	def __init__(self,port="/dev/ttyACM1",baudrate=115200):
		#allows you to avoid denied permissions on linux 
		if os.name == "posix":
			os.system("sudo chmod 666 "+port)
	
		self.port = port
		self.baudrate = baudrate
		
		#create and open the serial communication 
		self.ser = serial.Serial(port=self.port,baudrate=self.baudrate)
		self.ser.isOpen()
		
		#create the node, the publisher to the out of the vex and the subscriber for the command
		super().__init__('talker_vex')
		self.publisher_ = self.create_publisher(Vexmsg, 'out_serial', 10)
		
		self.subscription = self.create_subscription(
			Vexcommand,
			'in_serial',
			self.listener_callback,
			10
		)
		self.subscription

		# declare the timer for a 1000Hz Communication 		
		timer_period = 0.001
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		self.buffer_taille = 0
		self.buffer = []
		
		self.mutex = Lock()
		
	#function of the timer
	def timer_callback(self):
		msg = Vexmsg()
		
		#write of all command in the buffer
		with self.mutex:
				if(self.buffer_taille > 0):
					print(self.buffer_taille)
					print(self.buffer)
					#in the first time we send the number of command that we will send
					s = ("D"+str(self.buffer_taille)+"\n").encode()
					self.ser.write(s)
					for i in self.buffer:
						s = i.encode()
						self.ser.write(i.encode())
					self.buffer_taille = 0
					#when all command are send we empty the buffer 
					self.buffer = []
				else:
					#buffer empty
					self.ser.write(b'R\n')
		
		#read the frame from the vex and on the out_serial
		if self.ser.inWaiting() > 0:
			#in a first time we read the frame 
			t = self.ser.read(1).decode()
			if t != '\n':
				return
			tmp = ""
			
			if self.ser.inWaiting() > 0:
				t = self.ser.read().decode()
				tmp += t
			
			while t != '\r' and t != '\n':
				if self.ser.inWaiting() > 0:
					t = self.ser.read().decode()
					tmp += t	
	
			li = tmp[:-2].split("K")
			if(li[0]!="D"):
				return
			rotation_list = []
			motor_list = []
			inertial_list = []
			threewire_list = []
			try:
				#in a second time we create the vex message
				for i in range(1,len(li)):
					if(li[i][0]=='0'):
						screen = Vexscreen()
						li_s = li[i].split("#")
						screen.positionx = int(li_s[1])+1
						screen.positiony = int(li_s[2])
						screen.pressing = bool(int(li_s[3]))
						msg.screen = screen
					elif(li[i][0]=='1'):
						rotation_sensor = Vexrotationsensor()
						li_s = li[i].split("#")
						rotation_sensor.port = int(li[i][1])+1
						rotation_sensor.angle = float(li_s[1])
						rotation_sensor.velocity = float(li_s[2])
						rotation_list.append(rotation_sensor)
					elif(li[i][0]=='2'):
						motor = Vexmotor()
						li_s = li[i].split("#")
						motor.port = int(li[i][1])+1
						motor.isspining = bool(int(li_s[1]))
						motor.isdone = bool(int(li_s[2]))
						motor_list.append(motor)
					elif(li[i][0]=='3'):
						inertial = Vexinertial()
						li_s = li[i].split("#")
						inertial.port = int(li[i][1])+1
						inertial.accelerationx = float(li_s[1])
						inertial.accelerationy = float(li_s[2])
						inertial.accelerationz = float(li_s[3])
						inertial.heading = float(li_s[4])
						inertial.orientation_pitch = float(li_s[5])
						inertial.orientation_roll = float(li_s[6])
						inertial.orientation_raw = float(li_s[7])
						inertial.rotation = float(li_s[8])
						inertial_list.append(inertial)
					elif(li[i][0]=='4'):
						threewire = Vexthreewire()
						li_s = li[i].split('#')
						threewire.port = li[i][1]
						threewire.pressing = int(li_s[1])
						threewire_list.append(threewire)
					
				msg.motor = motor_list
				msg.rotationsensor = rotation_list
				msg.inertial = inertial_list
				msg.threewire = threewire_list
				self.publisher_.publish(msg)
			except Exception as e:
				print(type(e))
				print("erreur"+tmp)
		
	
	#funtion of the subscriber 
	#when with receive a message we put it in the buffer 	
	def listener_callback(self,msg):
		with self.mutex:
			s = ""
			s += msg.object
			s += msg.port 
			s += msg.command
			s += "#"
			s += msg.valeur
			s += "\n\n\n\n"
			self.buffer_taille += 1
			self.buffer.append(s)
			print(self.buffer_taille)
		
def main(args=None):
	rclpy.init(args=args)
	
	talker = Talker()
	
	rclpy.spin(talker)	
	
if __name__ == "__main__":
	main()
		
