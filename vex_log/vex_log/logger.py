import socket
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from threading import Lock

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand

from std_msgs.msg import Empty
from std_msgs.msg import Int64
from std_msgs.msg import Float64

class Logger(Node):
    def __init__(self, filename):
        super().__init__("logger")
        
        self.log_filename = "./data_logs/" + filename
        
        # Erase the file if it exists and create a new one (filename should be unique so it should not happen)
        with open(self.log_filename, 'w') as file:
            file.write("time,count\n")
        
        self.msg_received = []
        
        self.mutex = Lock()
        
        self.subscription = self.create_subscription(
            Float64,
            'count_topic',
            self.listener_callback,
            10
        )
        self.subscription
        
        self.terminate = self.create_subscription(
            Empty,
            'terminate_log',
            self.stop_node_callback,
            10
        )
        self.terminate
        
        # declare the timer for a 10Hz Data Logging		
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info("Logging in : " + self.log_filename)
        
    def listener_callback(self,msg):
        try:
            t = float(time.time());
            with self.mutex:
                self.msg_received.append((t, msg))
            #self.sock.recv(1024)
        except:
            self.get_logger().info("Sensor error: Connection severed with: " + str(self.addr))
            raise SystemExit 
        
        
    def timer_callback (self):
        with self.mutex:
            local_msg, self.msg_received = self.msg_received, []
        try:
            with open(self.log_filename, 'a') as file:
                for t, msg in local_msg:
                    file.write(str(t)+","+str(msg.data)+"\n")
        except:
            self.get_logger().info("Connection broken with: " + str(self.addr))
            raise SystemExit
        
    def stop_node_callback(self,msg):
        with self.mutex:
            # Destroy callbacks
            self.destroy_subscription(self.subscription)
            self.destroy_subscription(self.terminate)
            self.destroy_timer(self.timer)
            local_msg = self.msg_received
            try:
                with open(self.log_filename, 'a') as file:
                    for t, msg in local_msg:
                        file.write(str(t)+","+str(msg.angle)+","+str(msg.velocity)+"\n")
            except:
                pass
        raise SystemExit
                    
        

def main(args=None):
    rclpy.init(args=args)

    # socket.gethostbyname(os.environ["PARENTHOSTNAME"])
    # Automatic filename generation
    logger = Logger(datetime.now().strftime('%Y-%m-%d_H%H-%M-%S_log.csv'))
    
    try:
        rclpy.spin(logger)
    except SystemExit:
        pass   
    
    logger.destroy_node()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
