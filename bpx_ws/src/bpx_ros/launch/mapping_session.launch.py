"""매핑 세션 = 라이다(CustomMsg) + FAST-LIO. 관리자 웹 매핑 탭이 시작/정지한다.

  ros2 launch bpx_ros mapping_session.launch.py map_name:=office lidar_ip:=192.168.1.1XX
  ros2 launch bpx_ros mapping_session.launch.py map_name:=t source:=bag bag:=/path/custom_bag   # 라이다 없이 시험

source:=bag 이면 라이다 대신 CustomMsg bag 을 --clock 으로 재생 (use_sim_time).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _session(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)
    launch_dir = os.path.join(get_package_share_directory('bpx_ros'), 'launch')
    bag = arg('source') == 'bag'
    acts = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mapping.launch.py')),
        launch_arguments={'map_name': arg('map_name'), 'use_sim_time': 'true' if bag else 'false'}.items())]
    if bag:
        acts.append(ExecuteProcess(cmd=['ros2', 'bag', 'play', arg('bag'), '--clock', '--rate', arg('rate')],
                                   output='screen'))
    else:
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
            launch_arguments={'lidar_ip': arg('lidar_ip'), 'xfer_format': '1'}.items()))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='map'),
        DeclareLaunchArgument('source', default_value='live', description='live | bag'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.100'),
        DeclareLaunchArgument('bag', default_value=''),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=_session),
    ])
