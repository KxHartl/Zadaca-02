import math
from geometry_msgs.msg import TransformStamped, PoseStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from tf2_ros.buffer import Buffer



class FramePublisher(Node):

    def __init__(self):
        super().__init__('goal_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscriptionGoal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.handle_goal_2d,
            1
            )
        
        self.publisherTwist = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            1
            )
        self.subscriptionGoal
        self.timer = self.create_timer(0.1, self.broadcast_tf2)
        self.onecheck = 0
        self.pos_and_orient = [None, None, None, None, None, None, None]

    def broadcast_tf2(self):
        if(self.pos_and_orient[0] == None):
            pass
        else:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = '2d_goal_tf'
            t.transform.translation.x = self.pos_and_orient[0]
            t.transform.translation.y = self.pos_and_orient[1]
            t.transform.translation.z = self.pos_and_orient[2]
            t.transform.rotation.x = self.pos_and_orient[3]
            t.transform.rotation.y = self.pos_and_orient[4]
            t.transform.rotation.z = self.pos_and_orient[5]
            t.transform.rotation.w = self.pos_and_orient[6]

            self.tf_broadcaster.sendTransform(t)
            self.onecheck += 1

            if self.onecheck > 1:
                self.broadcast_vel()
            else:
                pass

    def handle_goal_2d(self, msg):
        self.pos_and_orient[0] = msg.pose.position.x
        self.pos_and_orient[1] = msg.pose.position.y
        self.pos_and_orient[2] = msg.pose.position.z
        self.pos_and_orient[3] = msg.pose.orientation.x
        self.pos_and_orient[4] = msg.pose.orientation.y
        self.pos_and_orient[5] = msg.pose.orientation.z
        self.pos_and_orient[6] = msg.pose.orientation.w

    def broadcast_vel(self):
        from_frame_rel = '2d_goal_tf'
        to_frame_rel = 'turtle1'
        t2 = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        rclpy.time.Time())

        msg2 = Twist()
        scale_rotation_rate = 1.0
        msg2.angular.z = scale_rotation_rate * math.atan2(
            t2.transform.translation.y,
            t2.transform.translation.x)

        scale_forward_speed = 0.5
        msg2.linear.x = scale_forward_speed * math.sqrt(
            t2.transform.translation.x ** 2 +
            t2.transform.translation.y ** 2)

        self.publisherTwist.publish(msg2)

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()