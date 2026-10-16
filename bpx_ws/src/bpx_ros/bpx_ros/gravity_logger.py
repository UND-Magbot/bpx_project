#!/usr/bin/env python3
"""매핑 시작 시 Livox IMU 로 중력 방향을 재서 <map>.gravity.json 으로 저장.

FAST-LIO 의 camera_init 은 출발 시 IMU 자세 그대로라 중력 정렬이 아니다.
이 값으로 scripts/align_map_gravity.py 가 지도를 수평으로 세운다.
로봇이 멈춰 있는 처음 1초(200 샘플) 평균을 쓴다. 흔들리면 경고만 하고 그대로 저장한다.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class GravityLogger(Node):
    def __init__(self):
        super().__init__('gravity_logger')
        self.out = self.declare_parameter('output', '').value
        self.n = self.declare_parameter('samples', 200).value
        self.acc = []
        self.create_subscription(Imu, self.declare_parameter('imu_topic', '/livox/imu').value,
                                 self.cb, qos_profile_sensor_data)

    def cb(self, m):
        if len(self.acc) >= self.n:
            return
        a = m.linear_acceleration
        self.acc.append((a.x, a.y, a.z))
        if len(self.acc) == self.n:
            self.save()

    def save(self):
        k = len(self.acc)
        g = [sum(v[i] for v in self.acc) / k for i in range(3)]
        norms = [math.sqrt(sum(c * c for c in v)) for v in self.acc]
        mean_n = sum(norms) / k
        std_n = math.sqrt(sum((x - mean_n) ** 2 for x in norms) / k)
        unit = 'g' if abs(mean_n - 1.0) < abs(mean_n - 9.81) else 'm/s2'
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, g[2] / math.sqrt(sum(c * c for c in g))))))
        rec = {'gravity_imu': g, 'norm': mean_n, 'norm_std': std_n, 'unit': unit,
               'samples': k, 'tilt_deg': tilt}
        with open(self.out, 'w') as f:
            json.dump(rec, f, indent=2)
        self.get_logger().info(f"중력 방향 저장 -> {self.out} (기울기 {tilt:.1f}°, 단위 {unit})")
        if std_n > 0.02 * mean_n:
            self.get_logger().warn(f"측정 중 흔들림이 큼 (|a| 표준편차 {std_n:.3f}) -> 정지 상태에서 매핑을 시작할 것")


def main():
    rclpy.init()
    node = GravityLogger()
    if not node.out:
        raise SystemExit("output 파라미터 필요")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
