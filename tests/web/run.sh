#!/bin/bash
set +u; source /home/undsw/bpx/setup/bpx_env.sh; export ROS_DOMAIN_ID=93
mkdir -p /home/undsw/bpx/lidar_ws/webtest
ros2 run bpx_ros bpx_bridge --ros-args -p backend:=fake -p allow_motion:=false > /home/undsw/bpx/lidar_ws/webtest/bridge.log 2>&1 &
# (map->odom 고정은 Nav2 시험 세션이 직접 발행)
exec ros2 run bpx_web bpx_web --ros-args -p port:=8090 -p test_bag_rate:=2.0 > /home/undsw/bpx/lidar_ws/webtest/web.log 2>&1
