#!/usr/bin/env python3
"""배터리·온도 장시간 기록 (충전 곡선, 소비 전력, 사용 시간 추정용). 로봇에 명령을 보내지 않는다.

  python3 scripts/battery_logger.py [간격초=10]      -> ~/bpx/logs/battery_YYYYMMDD_HHMMSS.csv
열: time, elapsed_s, battery_pct, current_a(+충전), charging, motion_state, mode, motor_temp_max, driver_temp_max
"""
import csv
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LOG_DIR = os.path.expanduser('~/bpx/logs')


class BatteryLogger(Node):
    def __init__(self, period):
        super().__init__('battery_logger')
        os.makedirs(LOG_DIR, exist_ok=True)
        self.path = os.path.join(LOG_DIR, time.strftime('battery_%Y%m%d_%H%M%S.csv'))
        self.f = open(self.path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['time', 'elapsed_s', 'battery_pct', 'current_a', 'charging', 'motion_state', 'mode',
                         'motor_temp_max', 'driver_temp_max'])
        self.t0 = time.time()
        self.st, self.diag = {}, {}
        self.create_subscription(String, '/bpx/status', lambda m: self.st.update(json.loads(m.data)), 10)
        self.create_subscription(String, '/bpx/diag', lambda m: self.diag.update(json.loads(m.data)), 10)
        self.create_timer(period, self.tick)
        self.get_logger().info(f"기록: {self.path} ({period}s 간격)")

    def tick(self):
        st, d = self.st, self.diag
        if not st.get('connected'):
            return
        mt, dt = d.get('motor_temp') or [], d.get('driver_temp') or []
        self.w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), round(time.time() - self.t0), st.get('battery'),
                         st.get('battery_current'), int(bool(st.get('charging'))), st.get('motion_state'),
                         st.get('mode'), max(mt) if mt else '', max(dt) if dt else ''])
        self.f.flush()


def main():
    rclpy.init()
    node = BatteryLogger(float(sys.argv[1]) if len(sys.argv) > 1 else 10.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()


if __name__ == '__main__':
    main()
