import rospy
from std_msgs.msg import Float32MultiArray
import random

class RobotPSO:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.position = [random.uniform(0, 10), random.uniform(0, 10)]
        self.velocity = [0, 0]
        self.best_position = list(self.position)
        self.best_concentration = -float('inf')
        self.global_best_position = [0, 0]

        self.pub = rospy.Publisher(f'/robot_{robot_id}/concentration', Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('/pso/global_best', Float32MultiArray, self.global_best_callback)

    def global_best_callback(self, msg):
        self.global_best_position = msg.data[:2]

    def measure_concentration(self, position):
        # Replace with actual concentration measurement
        return -((position[0]-5)**2 + (position[1]-5)**2) + random.uniform(-0.5, 0.5)

    def step(self):
        # PSO update
        w, c1, c2 = 0.5, 1.5, 1.5
        for i in range(2):
            r1, r2 = random.random(), random.random()
            self.velocity[i] = (w * self.velocity[i] +
                                c1 * r1 * (self.best_position[i] - self.position[i]) +
                                c2 * r2 * (self.global_best_position[i] - self.position[i]))
            self.position[i] += self.velocity[i]

        concentration = self.measure_concentration(self.position)
        if concentration > self.best_concentration:
            self.best_concentration = concentration
            self.best_position = list(self.position)

        msg = Float32MultiArray(data=self.position + [concentration])
        self.pub.publish(msg)

    def run(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('robot_pso_node')
    robot_id = rospy.get_param('~robot_id', 1)
    node = RobotPSO(robot_id)
    node.run()