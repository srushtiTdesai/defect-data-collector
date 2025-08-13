from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('defect_data_collector')
    cfg = os.path.join(pkg_share, 'config', 'collector.yaml')

    return LaunchDescription([
        Node(
            package='defect_data_collector',
            executable='simulator',
            name='pose_array_simulator',
            output='screen',
            parameters=[{
                'topic': '/defect_gen/defect_pose',
                'frame_id': 'world',
                'rate_hz': 10.0,
                'poses_per_msg': 5,
                'radius': 0.5,
            }]
        ),
        Node(
            package='defect_data_collector',
            executable='collector',
            name='pose_array_collector',
            output='screen',
            parameters=[cfg]
        ),
    ])
