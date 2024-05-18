import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import argparse
import yaml

class PoseListener(Node):
    import geometry_msgs.msg  # Import the necessary message type

    def __init__(self, file_path):
        super().__init__('goal_listener')
        self.file = open(file_path, 'w')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,  # Subscribe to the correct topic
            '/initialpose',  # Update the topic name to match the 2D pose estimate topic in RViz
            self.goal_callback,
            10)
    
    def goal_callback(self, msg):
        # This function is called whenever a new goal is published
        position = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        orientation = {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
        data = {
            'position': position,
            'orientation': orientation
        }
        self.get_logger().info("Saved goal to file: {}".format(data))
        yaml.dump([data], self.file, default_flow_style=False)

    def __del__(self):
        self.file.close()  # Close the file when the object is deleted

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', help='File path')
    args =  parser.parse_args()

    if not args.file:
        print("Please provide a file path using --file argument.")
        return

    rclpy.init()
    goal_listener = PoseListener(args.file)

    goal_listener.get_logger().info("Waiting for the first pose to be published...")

    try:
        rclpy.spin(goal_listener)
    except KeyboardInterrupt:
        pass

    goal_listener.destroy_node()
    
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()