import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from gadentools.Utils import Vector3

class GBest(Node):
    def __init__(self, position: Vector3, concentration: float, topic_name: str = '/gbest_topic'):
        # Initialize ROS 2 Node
        super().__init__('gbest_publisher_node')
        
        self.position = position
        self.concentration = concentration
        
        # Initialize ROS 2 publishers for position and concentration
        self.position_pub = self.create_publisher(Point, f'{topic_name}/position', 10)
        self.concentration_pub = self.create_publisher(Float32, f'{topic_name}/concentration', 10)
    
    def update_gbest(self, new_position: Vector3, new_concentration: float):
        if self.concentration < new_concentration:
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

    def get_gbest_data(self):
        # Define a callback that subscribes to the topic to get GBest data
        def callback(msg):
            self.position = msg.position
            self.concentration = msg.data
            self.get_logger().info(f"Received GBest Data: Position: {self.position}, Concentration: {self.concentration}")
            # Unsubscribe after receiving the data
            self.destroy_subscription(subscription)
        
        # Subscribe to the GBest data topic
        subscription = self.create_subscription(
            Point,  # You can customize the message type based on your needs
            '/gbest_topic/position',  # Adjust topic as needed
            callback,
            10
        )
        
        # Spin until data is received (or use some condition)
        rclpy.spin_once(self)  # This allows the callback to be called

def main(args=None):
    rclpy.init(args=args)
    gbest = GBest(Vector3(0.0, 0.0, 0.0), 0.0)
    gbest.get_gbest_data()  # Call to subscribe and get data
    rclpy.shutdown()

if __name__ == '__main__':
    main()
