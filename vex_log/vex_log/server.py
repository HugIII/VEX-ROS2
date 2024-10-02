import socket
import os
import time

import rclpy
from rclpy.node import Node

from threading import Lock

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand

from std_msgs.msg import Int32

class Server(Node):
    def __init__(self, hostname):
        super().__init__("server")
        
        
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.bind((hostname,6969))
        self.msg_received = []
        
        self.subscription = self.create_subscription(
            Vexrotationsensor,
            'out_rotationsensor_2',
            self.listener_callback,
            10
        )
        self.subscription
        
        # declare the timer for a 100Hz Socket Communication 		
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mutex = Lock()
        
        self.sock.listen(1)
        self.conn, self.addr = self.sock.accept()
        #self.sock.setblocking(False)
        
        self.get_logger().info("Connecting with: " + str(self.addr))
        
    def listener_callback(self,msg):
        try:
            t = float(time.time());
            with self.mutex:
                self.msg_received.append((t, msg))
            #self.sock.recv(1024)
        except:
            self.conn.close()
            self.sock.close()
            self.get_logger().info("Sensor error: Connection severed with: " + str(self.addr))
            raise SystemExit 
        
        
    def timer_callback (self):
        with self.mutex:
            local_msg, self.msg_received = self.msg_received, []
        try:
            for t, msg in local_msg:
                self.conn.send(str.encode(str(t)+"\n"+str(msg.angle)+"\n"+str(msg.velocity)))
        except:
            self.conn.close()
            self.sock.close()
            self.get_logger().info("Connection broken with: " + str(self.addr))
            raise SystemExit

def main(args=None):
    while True:
        rclpy.init(args=args)

        # socket.gethostbyname(os.environ["PARENTHOSTNAME"])
        server = Server('0.0.0.0')
        
        try:
            rclpy.spin(server)
        except SystemExit:
            pass   
        
        server.destroy_node()
        
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
