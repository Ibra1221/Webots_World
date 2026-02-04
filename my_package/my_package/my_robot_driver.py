import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, JointState
import math
import time
from std_msgs.msg import Int32, Float32

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        
        self.left_encoder = self.__robot.getDevice('left wheel sensor')
        self.right_encoder = self.__robot.getDevice('right wheel sensor')

        timestep = int(self.__robot.getBasicTimeStep())
        self.left_encoder.enable(timestep)
        self.right_encoder.enable(timestep)

        self.imu = self.__robot.getDevice('imu')
        self.imu.enable(int(self.__robot.getBasicTimeStep()))

        self.gyro = self.__robot.getDevice('gyro')
        self.gyro.enable(int(self.__robot.getBasicTimeStep()))

        self.accel = self.__robot.getDevice('accelerometer')
        self.accel.enable(int(self.__robot.getBasicTimeStep()))

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        self.last_time = time.time()
        self.__target_twist = Twist()

        # Initialize ROS2 node
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        

        # Subscribe to cmd_vel
        self.__node.create_subscription(TwistStamped, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.left_enc_pub = self.__node.create_publisher(Int32, 'left_wheel/encoder', 10)
        self.right_enc_pub = self.__node.create_publisher(Int32, 'right_wheel/encoder', 10)

        # Publisher for odometry
        self.__odom_pub = self.__node.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        
        self.imu_pub = self.__node.create_publisher(Imu, 'imu/data', 10)
        
        # ADD THIS: Joint state publisher
        self.joint_state_pub = self.__node.create_publisher(JointState, 'joint_states', 10)
        

    def __cmd_vel_callback(self, twist: TwistStamped):
        self.__target_twist = twist.twist

    def step(self):
        # Process cmd_vel messages
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Convert cmd_vel to wheel speeds
        left_vel = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_vel = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(left_vel)
        self.__right_motor.setVelocity(right_vel)

        # Compute odometry
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        v = WHEEL_RADIUS * (right_vel + left_vel) / 2.0
        w = WHEEL_RADIUS * (right_vel - left_vel) / (2.0 * HALF_DISTANCE_BETWEEN_WHEELS)

        self.theta += w * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.__node.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom_msg.twist.twist.linear = Vector3(x=v, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=w)

        self.__odom_pub.publish(odom_msg)

        roll, pitch, yaw = self.imu.getRollPitchYaw()
        q = quaternion_from_euler(roll, pitch, yaw)
        

        # Create IMU message
        imu_msg = Imu()
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.header.stamp = self.__node.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'
        
        ang_vel = self.gyro.getValues()
        lin_acc = self.accel.getValues()

        imu_msg.angular_velocity.x = ang_vel[0]
        imu_msg.angular_velocity.y = ang_vel[1]
        imu_msg.angular_velocity.z = ang_vel[2]

        imu_msg.linear_acceleration.x = lin_acc[0]
        imu_msg.linear_acceleration.y = lin_acc[1]
        imu_msg.linear_acceleration.z = lin_acc[2]

        # Publish IMU
        self.imu_pub.publish(imu_msg)


        left_pos = self.left_encoder.getValue()
        right_pos = self.right_encoder.getValue()

        # If you want ticks:
        TICKS_PER_REV = 4096
        left_ticks = int(left_pos * TICKS_PER_REV / (2 * math.pi))
        right_ticks = int(right_pos * TICKS_PER_REV / (2 * math.pi))

        self.left_enc_pub.publish(Int32(data=left_ticks))
        self.right_enc_pub.publish(Int32(data=right_ticks))  # OR Int32(right_ticks)

        # ADD THIS: Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.__node.get_clock().now().to_msg()
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state_msg.position = [left_pos, right_pos]
        joint_state_msg.velocity = [left_vel, right_vel]
        self.joint_state_pub.publish(joint_state_msg)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.__tf_broadcaster.sendTransform(t)