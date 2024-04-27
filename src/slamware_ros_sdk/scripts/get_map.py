import sys
import os
import rclpy
from slamware_ros_msgs.srv import SyncGetStcm

# write a ros2 python program to call sync_get_stcm service which delivers a slamware_ros_msgs/srv/SyncGetStcm message and extract the raw_stcm array and save it to a base filename specified on the command line and given an extension of .stcm
def save_stcm_to_file(filename):
    rclpy.init()
    node = rclpy.create_node('get_map_node')
    client = node.create_client(SyncGetStcm, 'sync_get_stcm')

    while not client.wait_for_service(timeout_sec=1.0):
        print('Service not available, waiting...')

    request = SyncGetStcm.Request()
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        response = future.result()
        raw_stcm = response.raw_stcm

        with open(filename, 'wb') as file:
            file.write(raw_stcm)
            print(f'STCM saved to {filename}')
    else:
        print('Failed to get STCM')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Please provide a filename')
        sys.exit(1)

    filename = sys.argv[1]
    if not filename.endswith('.stcm'):
        filename += '.stcm'

    save_stcm_to_file(filename)