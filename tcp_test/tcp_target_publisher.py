#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import socket

ARDUINO_HOST = '169.254.93.101'
ARDUINO_PORT = 10002


class TCPTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='tcp_target_publisher') # init node with the node name
        self.pub: rclpy.publisher.Publisher = self.create_publisher(
            msg_type=Float32,
            topic='linear_slider_target',
            qos_profile=10 # qos_profile or history depth
        )
        # Timer
        timer_period = 0.1
        self.timer: rclpy.timer.Rate = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        # Socket client
        self.pub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.pub_socket.connect((ARDUINO_HOST, ARDUINO_PORT))


        self.i = 0
        return
    
    def timer_callback(self):
        # Construct ROS msg, publish
        msg = Float32()

        self.i+= 1.0
        if self.i > 10:
            self.i=0.0
        
        msg.data = self.i
        self.pub.publish(msg)

        # log the info
        self.get_logger().info(f"Linear slider stepper target position: {msg.data}")
        
        # send data to Arduino server
        self.pub_socket.sendall(f"<{msg.data}>".encode())
        return
    



def main(args=None):
    rclpy.init(args=args)

    tcp_publisher = TCPTargetPublisher()

    rclpy.spin(tcp_publisher)

    tcp_publisher.pub_socket.close()


if __name__ == "__main__":
    main()