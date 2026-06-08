from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = join(
        get_package_share_directory('fluxeem_ros2_viewer'), 'params', 'param.yaml'
    )

    fluxeem_ros2_viewer_node = Node(
        package='fluxeem_ros2_viewer',
        executable='fluxeem_ros2_viewer_node',
        name='fluxeem_ros2_viewer_node',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        fluxeem_ros2_viewer_node,
    ])
