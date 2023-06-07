import rclpy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
HALF_DISTANCE_BETWEEN_WHEELS = 0.06
WHEEL_RADIUS = 0.034

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__left_motor = self.__robot.getDevice('left_motor')
        self.__right_motor = self.__robot.getDevice('right_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__target_twist = Twist()
        # self.__target_laser = LaserScan()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        # self.__node.create_subscription(LaserScan, 'base_scan', self.__laser_scan_callback, 1)
        
        # self.__laser_pub = self.__node.create_publisher(LaserScan, 'scan', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    # def __laser_scan_callback(self, laser):
    #     self.__target_laser = laser
    #     self.__target_laser.header.frame_id = "LDS-01"
    #     self.__target_laser.angle_min = 0.0
    #     self.__target_laser.angle_max = 2 * math.pi
    #     self.__target_laser.angle_increment = -self.__target_laser.angle_increment
    #     self.__target_laser.range_max = 15.0

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)

        # self.__laser_pub.publish(self.__target_laser)