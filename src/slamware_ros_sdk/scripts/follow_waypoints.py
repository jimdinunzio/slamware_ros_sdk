# a python script that takes a CLI argument --file file_name and opens and reads in this file with extension yaml
# in yaml format that specifies the position and orientation fields of a PoseWithCovarianceStamped message
# and stores the data in an array of PoseWithCovarianceStamped messages and sends them as waypoints to nav2
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import argparse
import yaml
from action_msgs.msg import GoalStatus

class WaypointSender(Node):
    def __init__(self, waypoints):
        super().__init__('waypoint_sender')
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.waypoints = waypoints
        self.get_logger().info('Waiting for server')        
        self.client.wait_for_server()

    def send_waypoints(self):
        self.get_logger().info('Sending waypoints')
        goal_msg = FollowWaypoints.Goal()
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = waypoint['position']['x']
            pose.pose.position.y = waypoint['position']['y']
            pose.pose.position.z = waypoint['position']['z']
            pose.pose.orientation.x = waypoint['orientation']['x']
            pose.pose.orientation.y = waypoint['orientation']['y']
            pose.pose.orientation.z = waypoint['orientation']['z']
            pose.header.frame_id = 'map'  # Assuming the waypoints are in the map frame
            goal_msg.poses.append(pose)

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        self.get_logger().info('Result: {}'.format(goal_handle.status))
        if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
            result = goal_handle.result()
            self.get_logger().info('Result: {}'.format(result))
            if hasattr(result, 'missed_waypoints'):
                self.get_logger().info('Missed Waypoints:')
                for waypoint in result.missed_waypoints:
                    self.get_logger().info(waypoint)

def main(args=None):
    rclpy.init(args=args)    

    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", help="Path to the YAML waypoints file")
    args = parser.parse_args()

    # Read the YAML file
    with open(args.file, 'r') as file:
        data = yaml.safe_load(file)

    # Process the data and store in an array
    waypoints = []
    for item in data:
        position = item['position']
        orientation = item['orientation']
        waypoint = {
            'position': position,
            'orientation': orientation
        }
        waypoints.append(waypoint)

    waypoint_sender = WaypointSender(waypoints)
    waypoint_sender.send_waypoints()
    waypoint_sender.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()