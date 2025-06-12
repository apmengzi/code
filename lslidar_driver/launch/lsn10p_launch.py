#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg
import os

def generate_launch_description():
    # 1. LiDAR 驱动节点配置
    driver_dir = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params', 'lidar_uart_ros2', 'lsn10p.yaml'
    )
    
    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir],
    )

    # 2. 添加 LaserScan 数据节流节点（降低频率到 10Hz）
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        arguments=['messages', '/scan', '5.0', '/scan_throttled'],
        output='screen',
    )

    # 3. 可选：RViz2 自动启动（带优化参数）
    rviz_config = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'rviz', 'lidar.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False,
            # 增大 LaserScan 插件的队列大小
            'LaserScan.queue_size': 30,
        }],
        output='screen',
    )

    return LaunchDescription([
        driver_node,
#        throttle_node,  # 启用节流
#        rviz_node,    # 按需启用
    ])
