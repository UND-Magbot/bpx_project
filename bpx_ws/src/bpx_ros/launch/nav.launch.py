"""BPX 표준 Nav2 (bringup.launch.py 와 함께 실행)

  ros2 launch bpx_ros nav.launch.py                       # 빈 지도 + map->odom 고정 (라이다 전 시험용)
  ros2 launch bpx_ros nav.launch.py map:=/path/map.yaml

라이다 전에는 위치추정이 없어 map->odom 을 항등으로 고정한다 (static_map_to_odom:=true).
라이다 + NDT 가 생기면 static_map_to_odom:=false 로 두고 map_to_odom 노드가 TF 를 발행.
장애물 레이어가 없으므로 실기 자율주행 금지 — fake 백엔드 시험 전용.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bpx_ros')
    nav2_share = get_package_share_directory('nav2_bringup')
    params = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'empty.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('static_map_to_odom', default_value='true'),

        Node(
            package='tf2_ros', executable='static_transform_publisher', name='static_map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            condition=IfCondition(LaunchConfiguration('static_map_to_odom')),
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[params, {'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': params, 'use_sim_time': 'false', 'autostart': 'true'}.items(),
        ),
    ])
