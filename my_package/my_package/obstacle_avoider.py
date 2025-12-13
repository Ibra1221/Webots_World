import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

# Detection threshold
OBSTACLE_THRESHOLD = 0.5  # meters - react earlier

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.left_sensor_value = float('inf')
        self.right_sensor_value = float('inf')

        self.create_subscription(Range, 'left_sensor', self.left_sensor_callback, 10)
        self.create_subscription(Range, 'right_sensor', self.right_sensor_callback, 10)

        # Timer to continuously send velocity commands
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # State tracking
        self.is_avoiding = False

    def left_sensor_callback(self, msg):
        self.left_sensor_value = msg.range
        self.get_logger().info(f'Left sensor: {msg.range:.3f}')

    def right_sensor_callback(self, msg):
        self.right_sensor_value = msg.range
        self.get_logger().info(f'Right sensor: {msg.range:.3f}')

    def control_loop(self):
        twist = Twist()
        
        # Check if obstacles detected
        left_obstacle = self.left_sensor_value < OBSTACLE_THRESHOLD
        right_obstacle = self.right_sensor_value < OBSTACLE_THRESHOLD
        
        # More aggressive thresholds for closer obstacles
        left_close = self.left_sensor_value < 0.25
        right_close = self.right_sensor_value < 0.25

        if left_close or right_close:
            # Very close obstacle - stop and turn sharply
            self.get_logger().warn(f'CLOSE OBSTACLE! L:{self.left_sensor_value:.3f} R:{self.right_sensor_value:.3f}')
            twist.linear.x = -0.08  # back up
            if left_close and right_close:
                twist.angular.z = 1.5  # turn sharply
            elif left_close:
                twist.angular.z = -1.5  # turn right sharply
            else:
                twist.angular.z = 1.5  # turn left sharply
            self.is_avoiding = True
            
        elif left_obstacle and right_obstacle:
            # Obstacle on both sides - turn around
            self.get_logger().info('Obstacle on both sides - turning')
            twist.linear.x = 0.0
            twist.angular.z = 1.2
            self.is_avoiding = True
            
        elif left_obstacle:
            # Obstacle on left - turn right
            self.get_logger().info(f'Obstacle on left ({self.left_sensor_value:.3f}m) - turning right')
            twist.linear.x = 0.05
            twist.angular.z = -1.0
            self.is_avoiding = True
            
        elif right_obstacle:
            # Obstacle on right - turn left
            self.get_logger().info(f'Obstacle on right ({self.right_sensor_value:.3f}m) - turning left')
            twist.linear.x = 0.05
            twist.angular.z = 1.0
            self.is_avoiding = True
            
        else:
            # No obstacles - go forward
            if self.is_avoiding:
                self.get_logger().info('Clear path - resuming forward motion')
                self.is_avoiding = False
            twist.linear.x = 0.12
            twist.angular.z = 0.0

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()