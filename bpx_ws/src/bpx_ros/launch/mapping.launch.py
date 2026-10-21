"""FAST-LIO 매핑 (Mid-360S). lidar.launch.py 와 함께 실행 (또는 mapping_session.launch.py 가 함께 띄움).

  ros2 launch bpx_ros mapping.launch.py map_name:=office
  ros2 service call /map_save std_srvs/srv/Trigger        # -> ~/bpx/maps/office/map.pcd
  python3 ~/bpx/scripts/align_map_gravity.py ~/bpx/maps/office/map.pcd   # -> map_level.pcd (위치추정용)

지도 하나 = 폴더 하나 (~/bpx/maps/<map_name>/). 관리자 웹 지도 탭이 이 구조를 읽는다.
로봇을 멈춘 상태로 시작할 것: 처음 1초 IMU 로 중력 방향을 재서 gravity.json 에 저장한다
(FAST-LIO 지도는 출발 자세 기준이라 기울어 있음).
FAST-LIO 자체 TF(camera_init -> body)로만 돈다. 로봇 TF 와는 위치추정 단계에서 연결.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP_ROOT = os.path.expanduser('~/bpx/maps')


def _fastlio(context):
    name = LaunchConfiguration('map_name').perform(context)
    out = os.path.join(MAP_ROOT, name)
    os.makedirs(out, exist_ok=True)
    share = get_package_share_directory('bpx_ros')
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    return [
        Node(
            package='fast_lio', executable='fastlio_mapping', name='laser_mapping', output='screen',
            parameters=[os.path.join(share, 'config', 'fastlio_mid360s.yaml'),
                        {'map_file_path': os.path.join(out, 'map.pcd'), 'use_sim_time': sim}],
        ),
        Node(
            package='bpx_ros', executable='gravity_logger', name='gravity_logger', output='screen',
            parameters=[{'output': os.path.join(out, 'gravity.json'), 'use_sim_time': sim}],
        ),
    ]


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory('fast_lio'), 'rviz', 'fastlio.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='map'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='rosbag 재생 시 true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_fastlio),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
