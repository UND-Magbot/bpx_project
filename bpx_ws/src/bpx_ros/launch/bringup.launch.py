"""BPX 기본 bringup: bpx_bridge + robot_state_publisher (+ 선택 RViz)

  ros2 launch bpx_ros bringup.launch.py                          # 실기, 읽기 전용
  ros2 launch bpx_ros bringup.launch.py allow_motion:=true       # 실기, 서기/주행 허용
  ros2 launch bpx_ros bringup.launch.py backend:=fake allow_motion:=true rviz:=true   # 로봇 없이
  foxglove:=true -> 원격 PC 에서 Foxglove 로 ws://<미니PC>:8765 접속 (보기 전용: 발행/서비스 호출 불가)
  web:=true      -> 관리자 웹 http://<미니PC>:8080 (조종 모드 전환·동작 버튼·조이스틱, 인증 없음)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bpx_ros')
    with open(os.path.join(share, 'urdf', 'bpx.urdf'), encoding='utf-8') as f:
        urdf = f.read()

    backend = LaunchConfiguration('backend')
    robot_ip = LaunchConfiguration('robot_ip')
    allow_motion = LaunchConfiguration('allow_motion')
    cmd_topic = LaunchConfiguration('cmd_topic')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='sdk', description='sdk | fake'),
        DeclareLaunchArgument('robot_ip', default_value='10.21.20.1'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('web', default_value='false'),
        DeclareLaunchArgument('web_port', default_value='8080'),

        Node(
            package='bpx_ros', executable='bpx_bridge', name='bpx_bridge', output='screen',
            parameters=[
                os.path.join(share, 'config', 'bridge.yaml'),
                {
                    'backend': backend,
                    'robot_ip': robot_ip,
                    'allow_motion': ParameterValue(allow_motion, value_type=bool),
                    'cmd_topic': cmd_topic,
                },
            ],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher', output='screen',
            parameters=[{'robot_description': urdf}],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(share, 'rviz', 'bpx.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # 원격 보기. WiFi 의 누구나 접속 가능하므로 clientPublish/services/parameters 는 끈다
        # (켜 두면 원격에서 /cmd_vel 발행·stand 호출이 가능해짐)
        Node(
            package='foxglove_bridge', executable='foxglove_bridge', output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int),
                'capabilities': ['connectionGraph', 'assets'],
            }],
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
        Node(
            package='bpx_web', executable='bpx_web', name='bpx_web', output='screen',
            parameters=[{'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
                         'cmd_topic': cmd_topic}],
            condition=IfCondition(LaunchConfiguration('web')),
        ),
    ])
