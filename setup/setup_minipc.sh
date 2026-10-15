#!/usr/bin/env bash
# BPX 미니PC 초기 설정 (Ubuntu 22.04 x86_64/arm64)
#   1) ROS 2 Humble + 필요한 패키지   2) bpx_sdk wheel   3) 방화벽 UDP 9873   4) 워크스페이스 빌드   5) ~/.bashrc
# 사용: 이 bpx 폴더를 미니PC로 복사(예: ~/bpx) 후   bash ~/bpx/setup/setup_minipc.sh
# 네트워크(NIC 고정 IP)는 자동으로 바꾸지 않음 -> setup/netplan-bpx.yaml 참고해 수동 적용.
set -euo pipefail
BPX_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARCH="$(uname -m)"
echo "[BPX] 폴더: $BPX_DIR  아키텍처: $ARCH"

. /etc/os-release
[ "$VERSION_ID" = "22.04" ] || { echo "Ubuntu 22.04 가 아님 ($VERSION_ID) — ROS 2 Humble 은 22.04 기준"; exit 1; }

echo "[1] ROS 2 Humble"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  sudo apt update && sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
  sudo apt install -y ros-humble-desktop
fi
sudo apt install -y \
  git python3-colcon-common-extensions python3-pip python3-numpy \
  ros-humble-robot-state-publisher ros-humble-tf2-tools ros-humble-teleop-twist-keyboard \
  ros-humble-pointcloud-to-laserscan

echo "[2] bpx_sdk (Python 3.10 wheel)"
case "$ARCH" in
  x86_64)  WHL="$BPX_DIR"/third_party/bpx_sdk/*x86_64*.whl ;;
  aarch64) WHL="$BPX_DIR"/third_party/bpx_sdk/*aarch64*.whl ;;
  *) echo "지원하지 않는 아키텍처: $ARCH"; exit 1 ;;
esac
python3 -m pip install --user $WHL
python3 -c "import bpx_sdk; print('  bpx_sdk', bpx_sdk.__version__)"

echo "[3] 방화벽: 로봇 상태 UDP 9873 허용 (ufw 켜져 있을 때만)"
if command -v ufw >/dev/null && sudo ufw status | grep -q "Status: active"; then
  sudo ufw allow 9873/udp
  sudo ufw allow 7895/udp   # JointLevelControl 고속 관절 상태 (나중에 사용)
fi

echo "[4] 워크스페이스 빌드"
set +u   # ROS setup.bash 는 미정의 변수를 참조함 -> -u 상태로 source 하면 중단됨
source /opt/ros/humble/setup.bash
set -u
(cd "$BPX_DIR/bpx_ws" && colcon build --symlink-install)

echo "[5] ~/.bashrc"
LINE="source $BPX_DIR/setup/bpx_env.sh"
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

echo
echo "완료. 새 터미널에서:"
echo "  ping 10.21.20.1                                   # 로봇 NIC 설정 후"
echo "  python3 $BPX_DIR/scripts/bpx_cmd.py check         # SDK 단독 확인"
echo "  ros2 launch bpx_ros bringup.launch.py             # 읽기 전용 브리지"
