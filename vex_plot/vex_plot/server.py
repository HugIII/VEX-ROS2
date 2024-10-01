import socket
import sys

import rclpy
from rclpy.node import Node

from vex_message.msg import Vexmsg
from vex_message.msg import Vexscreen
from vex_message.msg import Vexmotor
from vex_message.msg import Vexrotationsensor
from vex_message.msg import Vexcommand

from std_msgs.msg import Int32

class Server(Node):
    def __init__(self):
        super().__init__("server")
        
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.sock.connect(('localhost',6969))
        
        self.subscription = self.create_subscription(
            Vexrotationsensor,
            'out_rotationsensor_2',
            self.listener_callback,
            10
        )
        self.subscription
        
    def listener_callback(self,msg):
        try:
            self.get_logger().info("Publishing: angle = " + str(msg.angle))
            self.sock.send(str.encode(str(msg.angle)+"\n"+str(msg.velocity)))
            self.sock.recv(1024)
        except:
            self.sock.close()
            raise SystemExit 
        
        

def main(args=None):
    rclpy.init(args=args)
    
    server = Server()
    
    try:
        rclpy.spin(server)
    except SystemExit:
        pass   
    
    server.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
