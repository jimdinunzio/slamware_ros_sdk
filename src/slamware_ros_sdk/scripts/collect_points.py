from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
import sys
import argparse

class PointListener(Node):
    def __init__(self, file_path):
        super().__init__('point_listener')
        self.file = open(file_path, 'w')
        self.subscription = self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)

    def point_callback(self, msg):
        # This function is called whenever a new point is published
        print("Saved point to file: (", msg.point.x, msg.point.y, msg.point.z, ")")
        self.file.write(f"{msg.point.x},{msg.point.y},{msg.point.z}\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', help='File path')
    args = parser.parse_args()

    if not args.file:
        print("Please provide a file path using --file argument.")
        return

    rclpy.init()
    point_listener = PointListener(args.file)
    print("Waiting for points to be published...")
    try:
        rclpy.spin(point_listener)
    except KeyboardInterrupt:
        pass
    
    point_listener.file.close()
    point_listener.destroy_node()

    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
