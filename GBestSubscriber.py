import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gadentools.Utils import Vector3

class GBestSubscriber(Node):
    def __init__(self):
        super().__init__('gbest_subscriber_node')
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.position_subscription = self.create_subscription(
            Point,
            '/gbest_topic/position',
            self.position_callback,
            qos_profile
        )
        self.concentration_subscription = self.create_subscription(
            Float32,
            '/gbest_topic/concentration',
            self.concentration_callback,
            qos_profile
        )
        self.gbest_position = None
        self.gbest_concentration = None

    def position_callback(self, msg):
        self.gbest_position = Vector3(msg.x, msg.y, msg.z)

    def concentration_callback(self, msg):
        self.gbest_concentration = msg.data

def retrieve_gbest_position():
    subscriber = GBestSubscriber()
    while rclpy.ok() and subscriber.gbest_position is None:
        rclpy.spin_once(subscriber, timeout_sec=1.0)
    position = subscriber.gbest_position if subscriber.gbest_position is not None else Vector3(0, 0, 0)
    subscriber.destroy_node()
    return position

def retrieve_gbest_concentration():
    subscriber = GBestSubscriber()
    while rclpy.ok() and subscriber.gbest_concentration is None:
        rclpy.spin_once(subscriber, timeout_sec=1.0)
    concentration = subscriber.gbest_concentration if subscriber.gbest_concentration is not None else 0.0
    subscriber.destroy_node()
    return concentration