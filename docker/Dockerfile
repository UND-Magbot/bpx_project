# 로봇/미니PC 없이 bpx_ros 를 시험하는 ROS 2 Humble 컨테이너 (미니PC 와 같은 Ubuntu 22.04 / Python 3.10)
#   docker build -t bpx-ros -f docker/Dockerfile .        (bpx 폴더에서)
#   docker run --rm bpx-ros bash /bpx/docker/test_fake.sh
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-numpy ros-humble-robot-state-publisher ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY third_party/bpx_sdk /bpx/third_party/bpx_sdk
RUN pip3 install /bpx/third_party/bpx_sdk/*x86_64*.whl

COPY bpx_ws/src /bpx/bpx_ws/src
COPY scripts /bpx/scripts
COPY docker /bpx/docker
RUN . /opt/ros/humble/setup.sh && cd /bpx/bpx_ws && colcon build

ENV ROS_DOMAIN_ID=90
