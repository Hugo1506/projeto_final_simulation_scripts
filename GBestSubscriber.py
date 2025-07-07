import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point

class GBestSubscriber(Node):
    def __init__(self):
        super().__init__('gbest_subscriber_node')
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.subscription = self.create_subscription(
            Point,
            '/gbest_topic/position',
            self.listener_callback,
            qos_profile
        )
        self.gbest_position = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received GBest position: {msg}')
        self.gbest_position = msg

def retrieve_gbest_position():
    subscriber = GBestSubscriber()
    while rclpy.ok() and subscriber.gbest_position is None:
        rclpy.spin_once(subscriber, timeout_sec=1.0)
    position = subscriber.gbest_position
    subscriber.destroy_node()
    return position