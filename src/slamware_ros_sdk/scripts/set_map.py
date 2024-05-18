import sys
import os
import rclpy
from slamware_ros_msgs.srv import SyncSetStcm

# Write a ros2 program that takes a base filename (to which extension .stcm is added) on the command line and reads that file into memory and creates a slamware_ros_msgs/srv/SyncSetStcm message using that memory to set the  raw_stcm field and sends the message using the service call sync_set_stcm
def set_map():
    if len(sys.argv) < 2:
        print("Please provide a base filename as a command line argument.")
        return

    base_filename = sys.argv[1]
    stcm_filename = base_filename
    if not stcm_filename.endswith('.stcm'):
        stcm_filename += '.stcm'

    if not os.path.exists(stcm_filename):
        print(f"File {stcm_filename} does not exist.")
        return

    with open(stcm_filename, "rb") as file:
        raw_stcm = file.read()

    rclpy.init()
    node = rclpy.create_node('set_map_node')

    try:
        sync_set_stcm = node.create_client(SyncSetStcm, 'sync_set_stcm')
        sync_set_stcm.wait_for_service()
        request = SyncSetStcm.Request()
        request.raw_stcm = raw_stcm
        future = sync_set_stcm.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        print("Map set successfully.")
    except Exception as e:
        print(f"Failed to set map: {e}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    set_map()
