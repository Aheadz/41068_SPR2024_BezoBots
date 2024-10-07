#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class TurtlebotTeleopJoy(Node):

    def __init__(self):
        super().__init__('turtlebot_teleop_joy')

        self.linear_axis = self.declare_parameter('axis_linear', 1).value
        self.angular_axis = self.declare_parameter('axis_angular', 0).value
        self.deadman_button = self.declare_parameter('axis_deadman', 0).value
        self.linear_scale = self.declare_parameter('scale_linear', 0.5).value
        self.angular_scale = self.declare_parameter('scale_angular', 0.9).value

        self.deadman_pressed = False
        self.active = False
        self.last_published = Twist()

        # Create a custom QoS profile for higher priority
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos)

        self.timer = self.create_timer(0.02, self.publish)

    def joy_callback(self, joy):
        twist = Twist()
        twist.linear.x = self.linear_scale * joy.axes[self.linear_axis]
        twist.angular.z = self.angular_scale * joy.axes[self.angular_axis]
        self.last_published = twist
        if joy.buttons[self.deadman_button] == 1 and not self.active:
            self.active = True
        elif joy.buttons[self.deadman_button] == 1 and self.active:
            self.active = False
            

    def publish(self):
        if self.active:
            self.vel_pub.publish(self.last_published)
        else:
            self.vel_pub.publish(Twist())  # Publish zero twist to stop the robot

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TurtlebotTeleopJoy()
    
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

