#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from math import sin, cos
from tf_transformations import quaternion_from_euler

MAX_RANGE = 0.15

class LocalizationController(Node):
    def __init__(self):
        super().__init__('localization_controller')
        
        # ---------------- PUBLISHERS ----------------
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, '/my_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(Range, '/left_wheel/encoder', self.left_cb, 10)
        self.create_subscription(Range, '/right_wheel/encoder', self.right_cb, 10)
        
        # ---------------- STATE ----------------
        self.left_range = MAX_RANGE
        self.right_range = MAX_RANGE
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # ---------------- TIMER ----------------
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info("Localization + Obstacle Avoidance running")
    
    # ---------- SENSOR CALLBACKS ----------
    def left_cb(self, msg):
        self.left_range = msg.range
    
    def right_cb(self, msg):
        self.right_range = msg.range
    
    # ---------- MAIN LOOP ----------
    def loop(self):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        
        # --- Obstacle avoidance logic (SAME AS YOUR WORKING NODE) ---
        twist_stamped.twist.linear.x = 0.15
        if (
            self.left_range < 0.9 * MAX_RANGE
            or self.right_range < 0.9 * MAX_RANGE
        ):
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = -2.0
        
        self.cmd_pub.publish(twist_stamped)
        
        # --- Fake localization (keeps Webots happy) ---
        self.x += twist_stamped.twist.linear.x * 0.1
        
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)
        
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()