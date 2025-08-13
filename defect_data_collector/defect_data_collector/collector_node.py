import os
import time
import threading
from queue import SimpleQueue, Empty
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
    _HAS_PARQUET = True
except Exception:
    _HAS_PARQUET = False

import csv


def _expanduser(p):
    return os.path.abspath(os.path.expanduser(p))


class PoseArrayCollector(Node):
    def __init__(self):
        super().__init__('pose_array_collector')

        # Declare parameters (or loaded from YAML)
        self.declare_parameter('topic', '/defect_gen/defect_pose')
        self.declare_parameter('batch_size', 200)
        self.declare_parameter('max_flush_seconds', 2.0)
        self.declare_parameter('output_dir', '~/defect_logs')
        self.declare_parameter('file_basename', 'defect_pose_log')
        self.declare_parameter('format', 'parquet')
        self.declare_parameter('reliability', 'best_effort')
        self.declare_parameter('depth', 10)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        batch_size = self.get_parameter('batch_size').value
        max_flush_seconds = self.get_parameter('max_flush_seconds').value
        output_dir = _expanduser(self.get_parameter('output_dir').value)
        file_basename = self.get_parameter('file_basename').value
        fmt = self.get_parameter('format').value.lower()
        reliability = self.get_parameter('reliability').value.lower()
        depth = int(self.get_parameter('depth').value)

        os.makedirs(output_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if reliability == 'best_effort' else ReliabilityPolicy.RELIABLE),
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )

        self._queue = SimpleQueue()
        self._batch = []
        self._batch_size = int(batch_size)
        self._max_flush_seconds = float(max_flush_seconds)
        self._last_flush = time.time()
        self._fmt = 'parquet' if (fmt == 'parquet' and _HAS_PARQUET) else 'csv'
        if fmt == 'parquet' and not _HAS_PARQUET:
            self.get_logger().warn('pyarrow not available — falling back to CSV.')

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._parquet_path = os.path.join(output_dir, f"{file_basename}_{ts}.parquet")
        self._csv_path = os.path.join(output_dir, f"{file_basename}_{ts}.csv")

        self._csv_file = None
        self._csv_writer = None

        self._sub = self.create_subscription(PoseArray, topic, self._on_msg, qos)

        self._stop = threading.Event()
        self._writer = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer.start()
        self.get_logger().info(f"Collecting PoseArray on {topic} -> {self._fmt.upper()} in {output_dir}")

    def _on_msg(self, msg: PoseArray):
        now = self.get_clock().now().to_msg()
        for i, p in enumerate(msg.poses):
            row = {
                'stamp_sec': now.sec,
                'stamp_nanosec': now.nanosec,
                'array_index': i,
                'position_x': p.position.x,
                'position_y': p.position.y,
                'position_z': p.position.z,
                'orientation_x': p.orientation.x,
                'orientation_y': p.orientation.y,
                'orientation_z': p.orientation.z,
                'orientation_w': p.orientation.w,
                'frame_id': getattr(msg.header, 'frame_id', ''),
            }
            try:
                if self._queue.qsize() > self._batch_size * 5:
                    _ = self._queue.get_nowait()
                self._queue.put_nowait(row)
            except Exception:
                pass

        now_t = time.time()
        if now_t - self._last_flush >= self._max_flush_seconds:
            self._flush()

    def _writer_loop(self):
        while not self._stop.is_set():
            try:
                item = self._queue.get(timeout=0.2)
                self._batch.append(item)
                if len(self._batch) >= self._batch_size:
                    self._flush()
            except Empty:
                if (time.time() - self._last_flush) >= self._max_flush_seconds and self._batch:
                    self._flush()

    def _flush(self):
        if not self._batch:
            self._last_flush = time.time()
            return
        rows = self._batch
        self._batch = []
        self._last_flush = time.time()

        if self._fmt == 'parquet':
            table = pa.Table.from_pylist(rows)
            if not os.path.exists(self._parquet_path):
                pq.write_table(table, self._parquet_path)
            else:
                with pq.ParquetWriter(self._parquet_path, table.schema, use_dictionary=True) as writer:
                    writer.write_table(table)
        else:
            file_exists = os.path.exists(self._csv_path)
            if self._csv_file is None:
                self._csv_file = open(self._csv_path, 'a', newline='')
                self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=list(rows[0].keys()))
                if not file_exists:
                    self._csv_writer.writeheader()
            for r in rows:
                self._csv_writer.writerow(r)
                self._csv_file.flush()

    def destroy_node(self):
        self._stop.set()
        if self._writer.is_alive():
            self._writer.join(timeout=1.0)
        self._flush()
        if self._csv_file:
            self._csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PoseArrayCollector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
