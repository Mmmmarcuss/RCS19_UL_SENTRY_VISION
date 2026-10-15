"""
hw_bridge.launch.py — 启动 rm_hw_bridge 节点
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # 默认参数文件路径
    from ament_index_python.packages import get_package_share_directory
    config_dir = os.path.join(
        get_package_share_directory('rm_hw_bridge'), 'config', 'params.yaml')

    return LaunchDescription([
        # ---- 可覆盖参数声明 ----
        DeclareLaunchArgument(
            'serial_device', default_value='/dev/ttyUSB0',
            description='Serial port device path'),
        DeclareLaunchArgument(
            'baudrate', default_value='921600',
            description='Serial baudrate'),

        # ---- 节点 ----
        Node(
            package='rm_hw_bridge',
            executable='hw_bridge_node',
            name='rm_hw_bridge',
            output='screen',
            parameters=[
                config_dir,
                {
                    'serial_device': LaunchConfiguration('serial_device'),
                    'baudrate': LaunchConfiguration('baudrate'),
                },
            ],
            remappings=[
                # 可在此重映射话题名
            ],
        ),
    ])
