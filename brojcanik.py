#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class MyNode(Node):

    def __init__(self):
        super().__init__("brojcanik")
        self.get_logger().info("BROJCANIK SE ZAVRTIO")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback)

        self.cmd_vel_pub_ = self.create_publisher(Int16, 'broj', 10)

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