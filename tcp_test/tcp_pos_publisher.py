#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import socket

from .socketstreamer import SocketStreamReader

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 11412

class TCPPosPublisher(Node):
    def __init__(self) -> None:
        super().__init__(node_name='tcp_pos_publisher')
        self.sub: rclpy.subscriber.Subscriber = self.create_publisher(
            msg_type=Float32,
            topic='linear_slider_pos',
            qos_profile=10
        )

        # Timer
        timer_period=0.000001
        self.timer: rclpy.timer.Rate = self.create_timer(timer_period_sec=timer_period, callback=self.timer_callback)

        # Socket server
        self.sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        self.sub_socket.bind((SERVER_HOST, SERVER_PORT))
        self.sub_socket.listen(2)
        self.conn, self.addr = self.sub_socket.accept()
        if self.conn:
            self.get_logger().info(f"Listening on {self.addr}")
            self.socketstreamer = SocketStreamReader(self.conn)
        return
    

    def timer_callback(self):
        msg = Float32()
        try:
            # msg.data = float(self.conn.recv(10).decode().rstrip())
            msg.data = float(self.socketstreamer.readline().decode().rstrip())
        except ValueError as e:
            print(f"{e}: Could not convert msg type to float.")

        self.get_logger().info(f"Linear slider current position: {msg.data}")
        return
    


def main(args=None):
    rclpy.init(args=args)

    tcp_subscriber = TCPPosPublisher()

    rclpy.spin(tcp_subscriber)

    tcp_subscriber.sub_socket.close()

    return


if __name__ == "__main__":
    main()