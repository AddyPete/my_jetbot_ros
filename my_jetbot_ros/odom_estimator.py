import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
import tf_transformations

# Global variables used by the node

# Those should be parameters of the node
# The odometry frame name
ODOM_FRAME_NAME = 'odom'
# The base link frame name
BASE_LINK_FRAME_NAME = 'base_link'
# Rate of the output estimations
RATE_HZ = 20

# The estimated odometry topic name
ODOM_TOPIC_NAME = '/odom0'


class OdomEstimator(Node):
    def __init__(self):
        """This node estimates the current odometry based on the command
        velocity.
        """
        # Initializing the node
        super().__init__('odom_estimator')
        # The QOS profiles
        qos_profile = QoSProfile(depth=10)
        # Adding the odom tranaform boardcaster
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        # Creating the topic to publish the odometry readings
        self.odom_pub = self.create_publisher(
            Odometry, ODOM_TOPIC_NAME, qos_profile)
        # Logging that the node has started
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        # intializing some local varaiables necessary for the calculations
        self.last_time = self.get_clock().now()
        self.last_twist = Twist()
        self.last_twist.linear.x = 0.0
        self.last_twist.linear.y = 0.0
        self.last_twist.linear.z = 0.0
        self.last_twist.angular.x = 0.0
        self.last_twist.angular.y = 0.0
        self.last_twist.angular.z = 0.0
        # Current robot pose
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        # Subscribe to the velocity topic
        self.vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.vel_listener_callback, qos_profile)
        # Adding a timer to publish messages periodically
        self.timer = self.create_timer(1 / float(RATE_HZ), self.timer_callback)

    def timer_callback(self) -> None:
        """The timer callback to publish messages.
        """
        now_time = self.get_clock().now()
        self.make_odom_transform_and_msg(now_time)

    def vel_listener_callback(self, twist_msg: Twist):
        """A function that caches the last velocity message.

        Args:
            twist_msg (Twist): The last velocity message.
        """
        self.last_twist = twist_msg

    def make_odom_transform_and_msg(self, now_time) -> None:
        """Construct the odom transform and message and send them.

        Args:
            now_time: The current time message.
        """
        # Compute the current delta time in seconds
        delta_time = float((now_time - self.last_time).nanoseconds) / (10 ** 9)
        # Update the last time
        self.last_time = now_time
        # Update the robot pose
        self.theta += delta_time * self.last_twist.angular.z
        delta_distance = delta_time * self.last_twist.linear.x
        self.x_pos += delta_distance * math.cos(self.theta)
        self.y_pos += delta_distance * math.sin(self.theta)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        # Construct the odom message
        odom_msg = Odometry()
        odom_msg.header.frame_id = ODOM_FRAME_NAME
        odom_msg.child_frame_id = BASE_LINK_FRAME_NAME
        odom_msg.header.stamp = now_time.to_msg()
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.last_twist.linear.x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.last_twist.angular.z
        # Publish the odom message
        self.odom_pub.publish(odom_msg)
        # note: where do I publish the transform !!

        # t = TransformStamped()
        # t.header.stamp = now_time.to_msg()
        # t.header.frame_id = ODOM_FRAME_NAME
        # t.child_frame_id = BASE_LINK_FRAME_NAME

        # # Turtle only exists in 2D, thus we get x and y translation
        # # coordinates from the message and set the z coordinate to 0
        # t.transform.translation.x = self.x_pos
        # t.transform.translation.y = self.y_pos
        # t.transform.translation.z = 0.0

        # # For the same reason, turtle can only rotate around one axis
        # # and this why we set rotation in x and y to 0 and obtain
        # # rotation in z axis from the message
        # q = tf_transformations.quaternion_from_euler(0, 0, self.last_twist.angular.z)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        # # Send the transformation
        # self.broadcaster.sendTransform(t)
        
def main() -> None:
    # Initialize the ROS System
    rclpy.init(args=None)
    # Create the node
    odom_estimator = OdomEstimator()
    # Spin the node
    rclpy.spin(odom_estimator)
    # odom_estimator.destroy_node()
    # Shutdown ROS
    rclpy.shutdown()


if __name__ == '__main__':
    main()
