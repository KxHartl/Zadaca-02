#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class MyNode(Node):

    def __init__(self):
        super().__init__("kvadriranje_broja")
        self.get_logger().info("BROJCANIK SE PRATI")
        self.counter_ = 0
        self.broj_subscriber_ = self.create_subscription(Int16, 'broj', self.broj_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Int16, 'kvadrat_broja', 10)

    def broj_callback(self, msg: Int16):
        broj = msg.data
        x = Int16()
        x.data = broj*broj
        self.cmd_vel_pub_.publish(x)

    def timer_callback(self):
        self.counter_ += 1
        x = Int16()
        x.data = self.counter_
        self.cmd_vel_pub_.publish(x)

def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()