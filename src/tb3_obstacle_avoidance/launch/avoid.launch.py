import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 启动 turtlebot3_world 的 Gazebo 仿真
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # 启动避障节点
    avoid_node = Node(
        package='tb3_obstacle_avoidance',
        executable='avoid_node',
        name='avoid_node',
        output='screen',
    )

    return LaunchDescription([
        gazebo_world,
        avoid_node,
    ])
