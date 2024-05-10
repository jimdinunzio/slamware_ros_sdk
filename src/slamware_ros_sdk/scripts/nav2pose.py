import argparse
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('x', type=float, help='x position')
    parser.add_argument('y', type=float, help='y position')
    args = parser.parse_args()

    print("Navigating to ({}, {})".format(args.x, args.y))
    rclpy.init()

    node = rclpy.create_node('minimal_action_client')
    action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    # Wait for the action server to become available
    print('Waiting for action server...')
    action_client.wait_for_server()

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = args.x
    goal_msg.pose.pose.position.y = args.y
    goal_msg.pose.pose.orientation.w = 1.0
    future = action_client.send_goal_async(goal_msg)

    print('Sending Goal. Waiting for result')
    rclpy.spin_until_future_complete(node, future)

    result_future = future.result()

    if result_future.status == 0:
        print('Goal sent')
    else:
        print('Goal failed with status code: ' + str(result_future.status))

    rclpy.shutdown()

if __name__ == '__main__':
    main()