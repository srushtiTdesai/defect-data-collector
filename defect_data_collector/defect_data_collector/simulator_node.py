import math
import random
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

class PoseArraySimulator(Node):
    def __init__(self):
        super().__init__('pose_array_simulator')
        self.declare_parameter('topic', '/defect_gen/defect_pose')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('poses_per_msg', 5)
        self.declare_parameter('radius', 0.5)

        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.poses_per_msg = int(self.get_parameter('poses_per_msg').value)
        self.radius = float(self.get_parameter('radius').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=10)
        self.pub = self.create_publisher(PoseArray, self.topic, qos)

        self.t = 0.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(f"Simulating PoseArray on {self.topic} @ {self.rate_hz} Hz")

    def _tick(self):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        poses: List[Pose] = []
        for i in range(self.poses_per_msg):
            angle = (self.t * 0.2) + (2 * math.pi * i / max(1, self.poses_per_msg))
            r = self.radius * (0.8 + 0.4 * random.random())
            p = Pose()
            p.position.x = r * math.cos(angle) + 0.1 * random.uniform(-1, 1)
            p.position.y = r * math.sin(angle) + 0.1 * random.uniform(-1, 1)
            p.position.z = 0.02 * random.uniform(0, 1)
            p.orientation.x = random.uniform(-0.05, 0.05)
            p.orientation.y = random.uniform(-0.05, 0.05)
            p.orientation.z = random.uniform(-0.05, 0.05)
            p.orientation.w = 1.0
            poses.append(p)
        msg.poses = poses

        self.pub.publish(msg)
        self.t += 1.0 / self.rate_hz


def main():
    rclpy.init()
    node = PoseArraySimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
