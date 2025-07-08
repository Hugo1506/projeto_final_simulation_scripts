import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from gadentools.Utils import Vector3

class GBest(Node):
    def __init__(self, position: Vector3, concentration: float, topic_name: str = '/gbest_topic'):
        super().__init__('gbest_publisher_node')
        self.position = position
        self.concentration = concentration

        # Use Transient Local QoS so late subscribers get the last message
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.position_pub = self.create_publisher(Point, f'{topic_name}/position', qos_profile)
        self.concentration_pub = self.create_publisher(Float32, f'{topic_name}/concentration', qos_profile)
        # Publish initial values
        self.publish_position()
        self.publish_concentration()
    
    def update_gbest(self, new_position: Vector3, new_concentration: float):
        self.position = new_position
        self.concentration = new_concentration
        # Publish the updated position and concentration to the ROS topics
        self.publish_position()
        self.publish_concentration()

    def publish_position(self):
        position_msg = Point(x=self.position.x, y=self.position.y, z=self.position.z)
        self.position_pub.publish(position_msg)
        self.get_logger().info(f"Published Position: {position_msg}")

    def publish_concentration(self):
        concentration_msg = Float32(data=self.concentration)
        self.concentration_pub.publish(concentration_msg)
        self.get_logger().info(f"Published Concentration: {concentration_msg.data}")

    # Remove get_gbest_data() since it's not needed for publishing

def main(args=None):
    rclpy.init(args=args)
    gbest = GBest(Vector3(0.0, 0.0, 0.0), 0.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()