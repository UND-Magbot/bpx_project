"""Livox Mid-360S 드라이버 + base_link -> livox_frame static TF

  ros2 launch bpx_ros lidar.launch.py lidar_ip:=192.168.1.1XX      # XX = 라이다 SN 끝 두 자리
  ros2 launch bpx_ros lidar.launch.py lidar_ip:=... xfer_format:=0 # PointCloud2 (RViz/Foxglove 확인용)

드라이버 기본 설정(host 192.168.1.5, lidar 192.168.1.12)은 매뉴얼 기본값과 달라서
실행 때 JSON 을 생성한다. host = 미니PC enp4s0 192.168.1.50 (setup/net_bpx.sh).
xfer_format 1(CustomMsg) 은 FAST-LIO 입력용. 토픽: /livox/lidar, /livox/imu (IMU 200 Hz)
장착 위치(x y z roll pitch yaw)는 미정 -> 로봇에 올린 뒤 실측해서 넣는다.
"""
import json
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _driver(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    ports = {"cmd_data_port": 56100, "push_msg_port": 56200, "point_data_port": 56300,
             "imu_data_port": 56400, "log_data_port": 56500}
    cfg = {
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {
            "lidar_net_info": ports,
            "host_net_info": [dict({"host_ip": arg('host_ip')}, **{k: v + 1 for k, v in ports.items()})],
        },
        "lidar_configs": [{
            "ip": arg('lidar_ip'),
            "pcl_data_type": 1,
            "pattern_mode": 0,
            # 점군 자체는 livox_frame 그대로 두고, 장착 위치는 아래 static TF 로 표현한다
            "extrinsic_parameter": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "x": 0, "y": 0, "z": 0},
        }],
    }
    fd, path = tempfile.mkstemp(prefix='bpx_mid360s_', suffix='.json')
    with os.fdopen(fd, 'w') as f:
        json.dump(cfg, f, indent=2)

    return [
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen',
            parameters=[{
                'xfer_format': int(arg('xfer_format')),
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': path,
                'cmdline_input_bd_code': 'livox0000000001',
            }],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='base_to_livox',
            arguments=['--x', arg('x'), '--y', arg('y'), '--z', arg('z'),
                       '--roll', arg('roll'), '--pitch', arg('pitch'), '--yaw', arg('yaw'),
                       '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'],
        ),
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.100',
                              description='192.168.1.1XX, XX = SN 끝 두 자리'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.50'),
        DeclareLaunchArgument('xfer_format', default_value='1', description='0 PointCloud2 / 1 CustomMsg'),
    ]
    args += [DeclareLaunchArgument(k, default_value='0.0', description='base_link->livox_frame (미측정)')
             for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    return LaunchDescription(args + [OpaqueFunction(function=_driver)])
