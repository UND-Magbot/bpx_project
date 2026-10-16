# BPX ROS 2 환경 (~/.bashrc 에서 source)
BPX_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$BPX_DIR/lidar_ws/install/setup.bash" ] && source "$BPX_DIR/lidar_ws/install/setup.bash"   # livox_ros_driver2
[ -f "$BPX_DIR/bpx_ws/install/setup.bash" ] && source "$BPX_DIR/bpx_ws/install/setup.bash"

# Zishen 은 89 + Zenoh. BPX 는 우선 기본 DDS(FastDDS) + 별도 도메인으로 시작.
# Windows PC 에서 RViz 로 보려면 양쪽 ROS_DOMAIN_ID / RMW 를 같게.
export ROS_DOMAIN_ID=90
# export RMW_IMPLEMENTATION=rmw_zenoh_cpp   # Zishen 방식으로 갈 때
export BPX_ROBOT_IP=10.21.20.1
