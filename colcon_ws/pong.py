import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'latency_ping',
            self.listener_callback,
            10)

        self.pub = self.create_publisher(PoseStamped, 'latency_pong', 10)

    def listener_callback(self, msg):

        now = self.get_clock().now().to_msg()

        now_s = now.sec + 1e-9 * now.nanosec

        msg_s = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

        lag = now_s - msg_s

        self.get_logger().info('Delay: %f ms' % (lag * 1000))


        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
