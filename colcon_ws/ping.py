import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'latency_ping', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.create_subscription(PoseStamped, "latency_pong", self.listener_callback, 10)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')


    def listener_callback(self, msg):

        now = self.get_clock().now().to_msg()

        now_s = now.sec + 1e-9 * now.nanosec

        msg_s = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

        lag = now_s - msg_s

        self.get_logger().info('Round Trip Delay: %f ms' % (lag * 1000))




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
