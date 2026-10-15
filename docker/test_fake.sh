#!/usr/bin/env bash
# fake 백엔드로 브리지 전체 흐름 시험: 기동 -> 토픽 확인 -> 서기 -> 전진 -> odom 증가 -> 앉기
source /opt/ros/humble/setup.bash
source /bpx/bpx_ws/install/setup.bash
set -u
PASS=0; FAIL=0
ok(){ echo "  [PASS] $1"; PASS=$((PASS+1)); }
ng(){ echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }

echo "== SDK import"
python3 -c "import bpx_sdk; m=bpx_sdk.MotionLevelControl(); print('  bpx_sdk', bpx_sdk.__version__)" && ok "bpx_sdk import" || ng "bpx_sdk import"

echo "== 단독 스크립트 (fake)"
python3 /bpx/scripts/bpx_cmd.py check --fake | head -3 && ok "bpx_cmd check --fake" || ng "bpx_cmd check"

echo "== bringup (fake, allow_motion)"
ros2 launch bpx_ros bringup.launch.py backend:=fake allow_motion:=true >/tmp/launch.log 2>&1 &
LP=$!
sleep 6

topics=$(ros2 topic list)
for t in /odom/mc_odom /joint_states /imu/data /battery_state /bpx/status /robot_description /tf; do
  echo "$topics" | grep -qx "$t" && ok "topic $t" || ng "topic $t"
done

timeout 5 ros2 run tf2_ros tf2_echo odom fl_toe_link 2>/dev/null | grep -q Translation && ok "TF odom -> fl_toe_link" || ng "TF odom -> fl_toe_link"

ros2 service call /bpx_bridge/stand std_srvs/srv/Trigger | grep -q "success=True" && ok "stand 서비스" || ng "stand 서비스"
sleep 3
timeout 3 ros2 topic echo --once /bpx/status | grep -q '"mode": "ready"' && ok "mode=ready" || ng "mode=ready ($(timeout 3 ros2 topic echo --once /bpx/status | head -1))"

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" >/dev/null 2>&1 &
PP=$!
sleep 3
kill $PP
X=$(timeout 3 ros2 topic echo --once /odom/mc_odom --field pose.pose.position.x | head -1)
python3 -c "import sys; sys.exit(0 if float('$X')>0.5 else 1)" && ok "전진 후 odom x=$X" || ng "odom x=$X"

ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 9.0}}" >/dev/null 2>&1 &
PP=$!; sleep 2
V=$(timeout 3 ros2 topic echo --once /odom/mc_odom --field twist.twist.linear.x | head -1)
kill $PP
python3 -c "import sys; sys.exit(0 if abs(float('$V')-0.4)<1e-3 else 1)" && ok "속도 제한 0.4 (요청 9.0 -> $V)" || ng "속도 제한 v=$V"
sleep 1.5
V=$(timeout 3 ros2 topic echo --once /odom/mc_odom --field twist.twist.linear.x | head -1)
python3 -c "import sys; sys.exit(0 if abs(float('$V'))<1e-6 else 1)" && ok "cmd 끊김 -> 정지 (v=$V)" || ng "타임아웃 정지 v=$V"

ros2 topic pub --once /bpx/motion std_msgs/msg/String "{data: sit}" >/dev/null
sleep 3
timeout 3 ros2 topic echo --once /bpx/status | grep -q '"motion_state": 0' && ok "sit -> LyingDown" || ng "sit"

kill -TERM $LP; timeout 15 tail --pid=$LP -f /dev/null

echo "== 읽기 전용 거부"
ros2 launch bpx_ros bringup.launch.py backend:=fake >/tmp/launch2.log 2>&1 &
LP=$!; sleep 5
ros2 service call /bpx_bridge/stand std_srvs/srv/Trigger | grep -q "success=False" && ok "allow_motion=false 면 stand 거부" || ng "읽기 전용"
kill -TERM $LP; timeout 15 tail --pid=$LP -f /dev/null

echo "== 실제 SDK 백엔드, 로봇 없음 (크래시 없이 재시도해야 함)"
timeout 12 ros2 run bpx_ros bpx_bridge --ros-args -p backend:=sdk -p robot_ip:=10.21.20.1 >/tmp/sdk.log 2>&1
code=$?
[ $code -eq 124 ] && ok "로봇 없음에도 노드 유지 (timeout 종료)" || ng "노드 종료 code=$code: $(tail -3 /tmp/sdk.log)"
tail -4 /tmp/sdk.log | sed 's/^/    /'

echo
echo "결과: PASS=$PASS FAIL=$FAIL"
[ $FAIL -eq 0 ]
