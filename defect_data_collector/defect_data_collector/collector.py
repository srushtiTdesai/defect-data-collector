import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import pandas as pd
import os
from datetime import datetime

class PoseArrayCollector(Node):
    def __init__(self):
        super().__init__('pose_array_collector')
        self.declare_parameter('topic', '/defect_gen/defect_pose')
        self.declare_parameter('output_dir', '/root/defect_logs')
        self.declare_parameter('file_basename', 'defect_pose_log')
        self.declare_parameter('format', 'csv')

        topic = self.get_parameter('topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.file_basename = self.get_parameter('file_basename').value
        self.format = self.get_parameter('format').value

        os.makedirs(self.output_dir, exist_ok=True)
        self.subscriber = self.create_subscription(PoseArray, topic, self.listener_callback, 10)
        self.data = []

    def listener_callback(self, msg):
        for pose in msg.poses:
            self.data.append({
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            })
        if len(self.data) >= 10:
            self.flush_data()

    def flush_data(self):
        df = pd.DataFrame(self.data)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        file_path = os.path.join(self.output_dir, f"{self.file_basename}_{ts}.{self.format}")
        if self.format == 'csv':
            df.to_csv(file_path, index=False)
        else:
            df.to_parquet(file_path, index=False)
        self.get_logger().info(f"Saved {len(self.data)} poses to {file_path}")
        self.data = []

def main(args=None):
    rclpy.init(args=args)
    node = PoseArrayCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
