import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import sys


def callback(odom):
    global count
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    # use the current time for the stamp
    pose.header.stamp = odom.header.stamp
    pose.pose.pose.position = odom.pose.pose.position
    pose.pose.pose.orientation = odom.pose.pose.orientation
    pose.pose.covariance = odom.pose.covariance
    pub.publish(pose)
    time.sleep(0.25)
    count += 1
    if count == 10:
        node.destroy_subscription(subscription)
        sys.exit()

if __name__ == '__main__':
    rclpy.init()

    count = 0
    # Create a Node
    node = rclpy.create_node('send_init_pose')

    # Create a publisher for the initialpose topic
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    subscription = node.create_subscription(Odometry, '/odom', callback, 1)
    print("publishing initial pose 10 times")
    rclpy.spin(node)
