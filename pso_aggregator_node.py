import rospy
from std_msgs.msg import Float32MultiArray

class PSOAggregator:
    def __init__(self, num_robots):
        self.num_robots = num_robots
        self.best_concentration = -float('inf')
        self.global_best_position = [0, 0]
        self.subs = []
        for i in range(num_robots):
            sub = rospy.Subscriber(f'/robot_{i+1}/concentration', Float32MultiArray, self.callback)
            self.subs.append(sub)
        self.pub = rospy.Publisher('/pso/global_best', Float32MultiArray, queue_size=10)

    def callback(self, msg):
        x, y, concentration = msg.data
        if concentration > self.best_concentration:
            self.best_concentration = concentration
            self.global_best_position = [x, y]
            self.publish_global_best()

    def publish_global_best(self):
        msg = Float32MultiArray(data=self.global_best_position)
        self.pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pso_aggregator_node')
    num_robots = rospy.get_param('~num_robots', 2)
    aggregator = PSOAggregator(num_robots)
    aggregator.run()