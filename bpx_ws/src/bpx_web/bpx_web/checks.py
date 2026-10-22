"""점검 탭: 배터리 장기 기록·추정, 주행 점검 분석, 서기 전력 측정. 로봇 명령은 여기서 보내지 않는다."""
import csv
import glob
import math
import os
import time

LOG_DIR = os.path.expanduser('~/bpx/logs')
CAPACITY_AH = 5.8          # 판매처 사양 (8S2P, 28.8 V 공칭)
NOMINAL_V = 28.8


class BatteryLog:
    """10 초마다 ~/bpx/logs/battery_YYYYMMDD.csv 에 덧붙인다. 웹을 다시 켜도 이어진다."""
    COLS = ['ts', 'battery_pct', 'current_a', 'charging', 'motion_state', 'motor_temp_max', 'driver_temp_max']

    def __init__(self, keep_hours=12):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.keep = keep_hours * 3600
        self.rows = []
        for path in sorted(glob.glob(os.path.join(LOG_DIR, 'battery_????????.csv')))[-2:]:
            try:
                for r in csv.DictReader(open(path)):
                    self.rows.append([float(r['ts'])] + [None if r[c] == '' else float(r[c]) for c in self.COLS[1:]])
            except (OSError, ValueError, KeyError):
                continue
        self._trim()

    def _trim(self):
        cut = time.time() - self.keep
        self.rows = [r for r in self.rows if r[0] >= cut]

    def add(self, pct, cur, charging, ms, mt, dt):
        row = [round(time.time()), pct, cur, int(bool(charging)), ms, mt, dt]
        path = os.path.join(LOG_DIR, time.strftime('battery_%Y%m%d.csv'))
        new = not os.path.exists(path)
        with open(path, 'a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(self.COLS)
            w.writerow(['' if v is None else v for v in row])
        self.rows.append(row)
        self._trim()

    def payload(self, max_points=720):
        rows = self.rows
        step = max(1, len(rows) // max_points)
        return {'type': 'battery', 'rows': rows[::step], 'est': self.estimate()}

    def estimate(self):
        """최근 기록으로 추정: 충전 중이면 완충까지, 방전 중이면 남은 사용 시간."""
        rows = [r for r in self.rows if r[1] is not None]
        if len(rows) < 3:
            return {}
        last = rows[-1]
        pct, cur, charging = last[1], last[2], bool(last[3])
        recent = [r for r in rows if r[0] >= last[0] - 900 and bool(r[3]) == charging]   # 최근 15 분 같은 상태
        est = {'pct': pct, 'current_a': cur, 'charging': charging}
        if cur is not None:
            est['power_w'] = round(abs(cur) * NOMINAL_V)
        if len(recent) >= 6 and recent[-1][0] - recent[0][0] >= 300:
            # 최소제곱 기울기 (%/시간)
            t0 = recent[0][0]
            xs = [(r[0] - t0) / 3600 for r in recent]
            ys = [r[1] for r in recent]
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            den = sum((x - mx) ** 2 for x in xs)
            slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else 0
            est['rate_pct_h'] = round(slope, 1)
            if charging and slope > 0.5:
                est['full_in_min'] = round((100 - pct) / slope * 60)
            elif not charging and slope < -0.5:
                est['empty_in_min'] = round(pct / -slope * 60)
        if not charging and cur is not None and cur < -0.1:
            est['empty_in_min_by_current'] = round(pct / 100 * CAPACITY_AH / -cur * 60)
        return est


class DriveTest:
    """주행 점검 1 회: 명령(cmd)을 누르고 있는 동안 odom·전류를 모아 분석한다."""
    PRESETS = {
        'fwd': ('전진', (0.2, 0.0, 0.0)), 'back': ('후진', (-0.15, 0.0, 0.0)),
        'left': ('왼쪽 옆걸음', (0.0, 0.15, 0.0)), 'right': ('오른쪽 옆걸음', (0.0, -0.15, 0.0)),
        'ccw': ('좌회전', (0.0, 0.0, 0.5)), 'cw': ('우회전', (0.0, 0.0, -0.5)),
        'distance': ('거리 측정', None), 'power_idle': ('서기 대기 전력', (0.0, 0.0, 0.0)),
    }

    def __init__(self, name):
        self.name = name
        self.label, self.cmd = self.PRESETS[name]
        self.t0 = time.monotonic()
        self.odom = []          # (t, x, y, yaw, vx, vy, wz)
        self.current = []       # (t, A)
        self.cmd_first = None   # 0 아닌 명령을 처음/마지막으로 받은 시각 (분석 구간)
        self.cmd_last = None

    def mark_cmd(self):
        t = time.monotonic() - self.t0
        if self.cmd_first is None:
            self.cmd_first = t
        self.cmd_last = t

    def add_odom(self, x, y, yaw, vx, vy, wz):
        self.odom.append((time.monotonic() - self.t0, x, y, yaw, vx, vy, wz))

    def add_current(self, a):
        if a is not None:
            self.current.append((time.monotonic() - self.t0, a))

    def analyze(self, actual_m=None):
        dur = time.monotonic() - self.t0
        res = {'name': self.name, 'label': self.label, 'duration_s': round(dur, 1), 'cmd': self.cmd}
        cur = [a for _, a in self.current]
        if cur:
            res['current_avg_a'] = round(sum(cur) / len(cur), 2)
            res['power_w'] = round(abs(res['current_avg_a']) * NOMINAL_V)
        if len(self.odom) < 5:
            res['error'] = 'odom 데이터가 부족함 (너무 짧게 눌렀거나 로봇 미연결)'
            return res
        o0, o1 = self.odom[0], self.odom[-1]
        dx, dy = o1[1] - o0[1], o1[2] - o0[2]
        c, s = math.cos(o0[3]), math.sin(o0[3])
        res['dist_m'] = round(math.hypot(dx, dy), 3)
        res['fwd_m'], res['left_m'] = round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3)   # 시작 자세 기준
        res['yaw_deg'] = round(math.degrees(math.atan2(math.sin(o1[3] - o0[3]), math.cos(o1[3] - o0[3]))), 1)
        if self.cmd is None or self.name == 'power_idle':
            if actual_m:
                res['actual_m'] = actual_m
                res['odom_error_pct'] = round((res['dist_m'] - actual_m) / actual_m * 100, 1)
            return res
        axis = max(range(3), key=lambda i: abs(self.cmd[i]))
        want = self.cmd[axis]
        meas = [r[4 + axis] for r in self.odom if self.cmd_first is None or self.cmd_first <= r[0] <= (self.cmd_last or 0)]
        if self.cmd_first is None:
            res['error'] = '명령이 나가지 않음 (조종 모드·서기 상태 확인)'
            return res
        # 명령을 보낸 구간만: 처음 1 초(가속) 제외 ~ 마지막 명령 시각 (손을 놓은 뒤 정지 구간 제외)
        t_a, t_b = self.cmd_first + 1.0, self.cmd_last
        steady = [r[4 + axis] for r in self.odom if t_a <= r[0] <= t_b]
        res['cmd_s'] = round(t_b - self.cmd_first, 1)
        res['axis'] = ['vx', 'vy', 'wz'][axis]
        res['want'] = want
        if steady:
            res['measured'] = round(sum(steady) / len(steady), 3)
            res['ratio_pct'] = round(res['measured'] / want * 100)
        lag = next((r[0] - self.cmd_first for r in self.odom
                    if r[0] >= self.cmd_first and abs(r[4 + axis]) >= abs(want) * 0.5 and r[4 + axis] * want > 0), None)
        res['latency_s'] = round(lag, 2) if lag is not None else None
        # 방향: 속도 부호와 이동량 부호가 명령과 같아야 한다 (odom 규약 +x 앞, +y 왼쪽, +yaw 좌회전)
        moved = {0: res['fwd_m'], 1: res['left_m'], 2: res['yaw_deg']}[axis]
        mean_all = sum(meas) / len(meas)
        res['direction_ok'] = (mean_all * want > 0) and (moved * want > 0)
        return res
