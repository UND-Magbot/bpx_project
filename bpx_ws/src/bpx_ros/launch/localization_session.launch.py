"""위치추정 세션 = 라이다(PointCloud2) + NDT 위치추정 + 2D 지도 서버 (+ Nav2). 관리자 웹 지도 탭이 시작/정지.

  ros2 launch bpx_ros localization_session.launch.py map_dir:=~/bpx/maps/office lidar_ip:=192.168.1.1XX
  ros2 launch bpx_ros localization_session.launch.py map_dir:=... nav:=true          # Nav2 까지
  ros2 launch bpx_ros localization_session.launch.py map_dir:=... source:=bag bag:=/path/pc2_bag   # 라이다 없이

- 실기(live): Nav2 모드. map->odom 을 위치추정이 발행, odom->base_link 는 브리지(LegOdom).
  base_link->livox_frame 은 lidar.launch.py 가 발행하므로 위치추정 쪽 lidar TF 는 끈다.
- bag: 로봇 odom 이 없어 단독 모드. 실행 중인 브리지의 odom->base_link 와 부모가 겹치지 않도록
  base 프레임을 bag_base_link 로 바꾼다 (map->bag_base_link). lidar TF 는 위치추정 쪽이 항등으로 발행.
- 초기 위치: 지도 원점(매핑 시작점)에서 시작한다고 가정. 다르면 관리자 웹에서 "초기 위치 지정".
- IMU 사전적분은 끈다 (Livox IMU 가속도 단위가 g 라 설정 확인 전까지).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

LIDAR_POSE = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')   # 라이다 장착 위치·자세 -> lidar.launch.py
from launch_ros.actions import Node


def _session(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    share = get_package_share_directory('bpx_ros')
    map_dir = os.path.expanduser(arg('map_dir'))
    bag = arg('source') == 'bag'
    sim = 'true' if bag else 'false'
    lloc = os.path.join(get_package_share_directory('lidar_localization_ros2'), 'launch',
                        'mid360_legged_localization.launch.py')
    acts = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lloc), launch_arguments={
            'map_path': os.path.join(map_dir, 'map_level.pcd'),
            'cloud_topic': '/livox/lidar', 'imu_topic': '/livox/imu', 'use_sim_time': sim,
            'enable_map_odom_tf': 'false' if bag else 'true',
            'base_frame_id': 'bag_base_link' if bag else 'base_link',
            'publish_lidar_tf': 'true' if bag else 'false',
            'use_imu_preintegration': 'false', 'use_continuous_time_deskew': 'false',
            'set_initial_pose': 'true', 'initial_pose_x': '0.0', 'initial_pose_y': '0.0', 'initial_pose_z': '0.0',
            'initial_pose_qx': '0.0', 'initial_pose_qy': '0.0', 'initial_pose_qz': '0.0', 'initial_pose_qw': '1.0',
        }.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': os.path.join(map_dir, 'map.yaml'), 'use_sim_time': bag}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             output='screen', parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': bag}]),
    ]
    if bag:
        acts.append(ExecuteProcess(cmd=['ros2', 'bag', 'play', arg('bag'), '--clock', '--rate', arg('rate')],
                                   output='screen'))
    else:
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar.launch.py')),
            launch_arguments=dict({'lidar_ip': arg('lidar_ip'), 'xfer_format': '0'},
                                  **{k: arg('lidar_' + k) for k in LIDAR_POSE}).items()))
    if arg('nav') == 'true':
        nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        acts.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2), launch_arguments={
            'params_file': os.path.join(share, 'config', 'nav2.yaml'), 'use_sim_time': sim, 'autostart': 'true',
        }.items()))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/bpx/maps/map')),
        DeclareLaunchArgument('source', default_value='live', description='live | bag'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.100'),
        *[DeclareLaunchArgument('lidar_' + k, default_value='0.0', description='base_link->livox_frame') for k in LIDAR_POSE],
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('nav', default_value='false'),
        OpaqueFunction(function=_session),
    ])
