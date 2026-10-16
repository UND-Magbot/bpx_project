#!/usr/bin/env bash
# 라이다 스택 설치 (버전 고정)
#   1) Livox-SDK2 v1.3.1  -> /usr/local   (Mid-360S 지원 추가 버전)
#   2) livox_ros_driver2 1.2.6 -> bpx/lidar_ws (Mid-360S 지원 추가 버전, ROS 2 Humble)
#   3) FAST-LIO (hku-mars ROS2 브랜치, 태그 없음 -> 커밋 고정) -> bpx/lidar_ws
# 사용: bash ~/bpx/setup/install_livox.sh    (sudo 필요: apt, make install)
# 소스는 git 에 넣지 않고 이 스크립트가 태그/커밋으로 받아 빌드한다. lidar_ws/ 는 .gitignore.
set -euo pipefail
SDK_TAG=v1.3.1
DRV_TAG=1.2.6
FASTLIO_COMMIT=a4743b0954   # ROS2 브랜치 2025-01-15 (blind 파라미터 수정 포함)
BPX_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="$BPX_DIR/lidar_ws"

echo "[1] 의존 패키지"
sudo apt install -y cmake build-essential git libpcl-dev libapr1-dev libeigen3-dev \
  ros-humble-pcl-conversions ros-humble-pcl-ros ros-humble-rosbag2

echo "[2] Livox-SDK2 $SDK_TAG"
if [ "$(cat /usr/local/share/livox_sdk2_version 2>/dev/null)" != "$SDK_TAG" ]; then
  TMP="$(mktemp -d)"
  git clone -q --depth 1 -b "$SDK_TAG" https://github.com/Livox-SDK/Livox-SDK2.git "$TMP/Livox-SDK2"
  cmake -S "$TMP/Livox-SDK2" -B "$TMP/build" -DCMAKE_BUILD_TYPE=Release
  make -C "$TMP/build" -j"$(nproc)"
  sudo make -C "$TMP/build" install
  echo "$SDK_TAG" | sudo tee /usr/local/share/livox_sdk2_version >/dev/null
  sudo ldconfig
  rm -rf "$TMP"
else
  echo "  이미 설치됨"
fi

echo "[3] livox_ros_driver2 $DRV_TAG -> $WS"
mkdir -p "$WS/src"
if [ ! -d "$WS/src/livox_ros_driver2" ]; then
  git clone -q --depth 1 -b "$DRV_TAG" https://github.com/Livox-SDK/livox_ros_driver2.git "$WS/src/livox_ros_driver2"
fi
# 제조사 build.sh 는 lidar_ws 의 build/install 을 통째로 지우므로 쓰지 않고 같은 준비만 직접 한다
(cd "$WS/src/livox_ros_driver2" && cp -f package_ROS2.xml package.xml && rm -rf launch && cp -r launch_ROS2 launch)

echo "[4] FAST-LIO $FASTLIO_COMMIT -> $WS"
if [ ! -d "$WS/src/FAST_LIO" ]; then
  git clone -q -b ROS2 https://github.com/hku-mars/FAST_LIO.git "$WS/src/FAST_LIO"
  git -C "$WS/src/FAST_LIO" -c advice.detachedHead=false checkout -q "$FASTLIO_COMMIT"
  git -C "$WS/src/FAST_LIO" submodule update -q --init --recursive
fi

echo "[5] lidar_ws 빌드"
set +u   # ROS setup.bash 는 미정의 변수를 참조함
source /opt/ros/humble/setup.bash
set -u
(cd "$WS" && MAKEFLAGS="-j8" colcon build --symlink-install \
   --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble)

echo
echo "완료. 새 터미널에서: ros2 launch bpx_ros lidar.launch.py lidar_ip:=192.168.1.1XX"
echo "  (XX = 라이다 SN 끝 두 자리, 라이다 전원 12 V / 랜선은 enp4s0)"
