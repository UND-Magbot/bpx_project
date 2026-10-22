#!/usr/bin/env python3
"""IMU 축 방향 확인 (로봇 명령 없음). 손으로 로봇 앞쪽/왼쪽을 들어 올리는 동안 자세·가속도를 기록.
  python3 scripts/imu_tilt_probe.py [초=120]  -> ~/bpx/logs/imu_tilt_*.csv
"""
import csv, math, os, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

rclpy.init(); n = Node('imu_tilt_probe')
os.makedirs(os.path.expanduser('~/bpx/logs'), exist_ok=True)
path = os.path.expanduser(time.strftime('~/bpx/logs/imu_tilt_%Y%m%d_%H%M%S.csv'))
f = open(path, 'w', newline=''); w = csv.writer(f)
w.writerow(['t', 'roll_deg', 'pitch_deg', 'yaw_deg', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
t0 = time.time(); last = [0.0]
def cb(m):
    now = time.time()
    if now - last[0] < 0.1: return          # 10 Hz 로 기록
    last[0] = now
    q = m.orientation
    roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    a, g = m.linear_acceleration, m.angular_velocity
    w.writerow([round(now - t0, 2), round(roll, 2), round(pitch, 2), round(yaw, 2),
                round(a.x, 3), round(a.y, 3), round(a.z, 3), round(g.x, 3), round(g.y, 3), round(g.z, 3)]); f.flush()
n.create_subscription(Imu, '/imu/data', cb, qos_profile_sensor_data)
end = t0 + (float(sys.argv[1]) if len(sys.argv) > 1 else 120)
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
f.close(); print(path)
