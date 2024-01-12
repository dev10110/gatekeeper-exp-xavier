import sys

from nvblox_msgs.srv import FilePath
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from rclpy.time import Time
from rclpy.duration import Duration

import time


class ServiceSavePly(Node):

    def __init__(self):
        super().__init__('service_save_ply')

        # parameters
        self.declare_parameter('save_path', rclpy.Parameter.Type.STRING)
        self.save_path  = self.get_parameter('save_path').value
        
        self.declare_parameter('save_time_seconds', 60.0) # defaults to 1 min
        self.save_time = self.get_parameter('save_time_seconds').value

        # time to collect the data
        self.target_time = None
        self.duration = Duration(seconds = self.save_time) 

        # create the subscriber
        self.sub = self.create_subscription(
                CameraInfo,
                "/camera/depth/camera_info",
                self.camera_callback,
                10)

        # create the service
        self.req = FilePath.Request()
        self.client = self.create_client(FilePath, '/nvblox_node/save_ply_with_rotation')

        # wait for the service to exist
        while not self.client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not running, waiting...')
        self.get_logger().info("found the service, .. waiting for the right time to trigger the service")


    def send_request(self):
        self.req.file_path = self.save_path # "/workspaces/isaac_ros-dev/maps/run_1/clean_maps"
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f" request returned: {self.future.result()}")
        return self.future.result()

    def camera_callback(self, msg):
        
        # based on the first message received, determine the time 
        if self.target_time is None:
            stamp = Time.from_msg(msg.header.stamp)
            self.target_time = stamp + self.duration
            self.get_logger().info(f"Got Image! Set target time to {self.target_time}")


        elif self.target_time is not None:
            if Time.from_msg(msg.header.stamp) < self.target_time:

                # waiting
                remaining = self.target_time - Time.from_msg(msg.header.stamp)

                print_msg = f"waiting for target time. {remaining.nanoseconds * 1e-9:0.1f} s..."
                self.get_logger().info(print_msg, throttle_duration_sec=2)
                return
            else:

                self.get_logger().info("HIT TARGET TIME! sending request")
                response = self.send_request()
                
                if response.success:
                    self.get_logger().info("Successfully saved ply")
                else:
                    self.get_logger().warn("FAILED TO SAVE PLY")

                # either way its time to exit
                time.sleep(2)

                self.destroy_node()
                



def main():

    rclpy.init()

    client = ServiceSavePly()

    # response = client.send_request()
    # client.get_logger().info(f" Request: {response.success}")

    rclpy.spin(client)


    client.destroy_node();

    rclpy.shutdown()


if __name__ == "__main__":
    main();
