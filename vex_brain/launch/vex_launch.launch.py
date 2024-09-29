########################################################
#                                                      #
#Autor: BLAYES Hugo                                    #
#Created: 8/7                                          #
#Version: 1.0.0                                        #
#Description: ROS2 Launch for vex communication        #
#                                                      #
########################################################

import serial
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
	nodes = []
	obj_li = {"motor":"1","rotation_sensor":"2","inertial":"3","threewire":"4"}
	#if screen is true we have a node for the screen
	screen = False
	
	#open the config for dynamical communication
	f = open("./vex_brain/vex_config")
	

	content = f.read().split("\n")[:-1]
	
	#setting up the configuration frame
	frame = []
	frame.append("I"+str(len(content))+"\n")
	for i in content:
		s = i.split("=")
		st = s[0]
		st += "#"
		st += obj_li[s[1]]
		st += "\n"
		frame.append(st)
	
	print(frame)

	#communication with the vex to send the config	
	os.system("sudo chmod 666 /dev/ttyACM1")
	ser = serial.Serial(port="/dev/ttyACM1",baudrate=115200)
	ser.isOpen()
	
	for i in frame:	
		ser.write(i.encode())
	
	ser.close()
	f.close()
	
	#reppon the file to active the right node 
	f = open("./vex_brain/vex_config")
	
	content = f.read().split("\n")
	
	for i in content:
		s = i.split("=")
		if len(s) == 2:
			port = s[0]
			obj = s[1]
			if obj not in obj_li.keys():
				pass
			
			#create this param for transmit the port to the node
			param = DeclareLaunchArgument(
				'port'+port,
				default_value=port
			)
			
			nodes.append(param)
			
			#create the node according to the config
			nodes.append(Node(package='vex_brain',executable=obj,parameters=[{'port':LaunchConfiguration('port'+port)}],output='screen'))
	
	f.close()
	
	nodes.append(Node(package='vex_brain',executable='talker',output='screen'))
	if(screen==True):
		nodes.append(Node(package='vex_brain',executable='screen',output='screen'))
	
	return LaunchDescription(nodes)
