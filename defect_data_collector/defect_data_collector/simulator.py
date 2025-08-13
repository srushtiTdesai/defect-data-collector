import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import random

class PoseArraySimulator(Node):
    def __init__(self):
        super().__init__('pose_array_simulator')
        self.declare_parameter('topic', '/defect_gen/defect_pose')
        topic = self.get_parameter('topic').value
        self.publisher_ = self.create_publisher(PoseArray, topic, 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = PoseArray()
        for _ in range(5):
            pose = Pose()
            pose.position.x = random.uniform(-1.0, 1.0)
            pose.position.y = random.uniform(-1.0, 1.0)
            pose.position.z = random.uniform(-1.0, 1.0)
            msg.poses.append(pose)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseArraySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
