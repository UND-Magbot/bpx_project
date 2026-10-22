#!/usr/bin/env python3
"""BPX 로봇 관리자 웹 서버 (FastAPI + rclpy 한 프로세스)

  ros2 run bpx_web bpx_web --ros-args -p port:=8080
  브라우저: http://<미니PC>:8080

  화면 -> 서버 (WebSocket /ws, JSON)
    {"type":"teleop","vx":..,"vy":..,"wz":..}   조이스틱 (10 Hz 이상으로 계속 보냄, 끊기면 서버가 0 을 보냄)
    {"type":"action","name":"stand|sit|damping|zero|walk|run"}
    {"type":"restart_bridge"}                      bpx-bridge 서비스 재시작 (이 웹도 함께 재시작됨)
    {"type":"motion","enable":true|false}         조종 모드 (브리지 ~/set_motion)
    {"type":"goal","x":..,"y":..,"yaw":..} / {"type":"cancel_goal"}   Nav2 (지도 좌표계에서만)
  서버 -> 화면: {"type":"state",...} 5 Hz, {"type":"map",...} 지도가 바뀔 때, {"type":"event","text":..}

안전: 조이스틱 속도는 서버에서도 브리지와 같은 상한으로 자른다. 조이스틱 입력이 0.3 s 끊기면 0 을 한 번 보내고 멈춘다.
로봇이 실제로 명령을 따르는지는 브리지의 조종 모드(allow_motion)가 최종 결정한다. 접속 인증 없음 (사내 시험망 전용).
"""
import asyncio
import base64
import json
import math
import os
import shutil
import subprocess
import threading
import time
from collections import deque

import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, JointState, PointCloud2
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

from bpx_web.checks import BatteryLog, DriveTest
from bpx_web.maps import MapStore, pool_grid
from bpx_web import settings as cfg
from bpx_web.procs import SessionManager
from bpx_web.sysinfo import SysInfo

MAP_MAX_CELLS = 3_000_000      # 화면으로 보내는 2D 지도 최대 칸 수 (넘으면 줄임)
PREVIEW_RES = 0.2              # 매핑 실시간 미리보기 격자 (m)
PREVIEW_Z = (0.3, 1.8)         # 미리보기 높이 범위: 라이다 기준 아래 0.3 m ~ 위 1.8 m (수평 보정 후)
BAG_DIR = os.path.expanduser('~/bpx/lidar_ws/bags')

STATIC_DIR = os.path.join(get_package_share_directory('bpx_web'), 'static')
TELEOP_TIMEOUT_S = 0.3
GOAL_STATUS = {1: '수락됨', 2: '진행 중', 3: '취소 중', 4: '도착', 5: '취소됨', 6: '실패'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def clamp(v, lim):
    return max(-lim, min(lim, float(v)))


class WebNode(Node):
    def __init__(self):
        super().__init__('bpx_web')
        p = self.declare_parameter
        self.port = p('port', 8080).value
        bridge = p('bridge_node', '/bpx_bridge').value
        self.max_v = (p('max_vx', 0.4).value, p('max_vy', 0.25).value, p('max_wz', 1.0).value)
        self.lock = threading.Lock()
        self.events = deque(maxlen=50)

        self.status = {}
        self.battery = {}
        self.odom = {}
        self.map_msg = None
        self.map_version = 0
        self.path = deque(maxlen=3000)
        self.path_frame = None
        self.pose = None                      # (frame, x, y, yaw)
        self.goal = None
        self.goal_handle = None
        self.goal_state = ''
        self.teleop = (0.0, 0.0, 0.0)
        self.teleop_t = 0.0
        self.teleop_active = False

        self.create_subscription(String, '/bpx/status', self.status_cb, 10)
        self.create_subscription(String, '/bpx/diag', self.diag_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.diag = {}
        self.joints = {}
        self.history = deque(maxlen=1800)       # 1 Hz x 30 분
        self.sys = SysInfo()
        self.sysinfo = {}
        # 매핑 / 지도 관리
        self.test_bag_custom = p('test_bag_custom', os.path.join(BAG_DIR, 'aist_custom')).value
        self.test_bag_pc2 = p('test_bag_pc2', os.path.join(BAG_DIR, 'rosbag2_2024_04_16-14_17_01')).value
        self.test_bag_rate = p('test_bag_rate', 1.0).value
        self.maps = MapStore(log=lambda t: self.get_logger().info(t))
        self.sessions = SessionManager()
        self.reset_preview()
        self.create_subscription(PointCloud2, '/cloud_registered', self.cloud_cb, 5)
        self.create_subscription(Odometry, '/Odometry', self.lio_odom_cb, 10)
        self.map_save_cli = self.create_client(Trigger, '/map_save')
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 점검 탭
        self.batlog = BatteryLog()
        self._bat_tick = 0
        self.imu = {}
        self.test = None
        self.test_results = []
        self.create_subscription(Imu, '/imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom/mc_odom').value, self.odom_cb, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic', '/cmd_vel').value, 10)
        self.trig = {n: self.create_client(Trigger, f'{bridge}/{n}')
                     for n in ('stand', 'sit', 'damping', 'zero', 'walk', 'run')}
        self.motion_cli = self.create_client(SetBool, f'{bridge}/set_motion')
        self.param_cli = self.create_client(SetParameters, f'{bridge}/set_parameters')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.create_timer(0.05, self.teleop_tick)
        self.create_timer(0.2, self.pose_tick)
        self.create_timer(1.0, self.history_tick)
        self.create_timer(2.0, self.sys_tick)

    # ---------------- 구독 ----------------
    def status_cb(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            prev = self.status
            self.status = dict(st, _t=time.time())
            if self.test:
                self.test.add_current(st.get('battery_current'))
            if st.get('limits'):                        # 조이스틱 상한 = 브리지의 현재 상한
                self.max_v = tuple(float(v) for v in st['limits'][:3])
        for k, label in (('connected', '로봇 연결'), ('mode', '브리지 모드'), ('allow_motion', '조종 모드')):
            if prev and prev.get(k) != st.get(k):
                self.event(f"{label}: {prev.get(k)} → {st.get(k)}")

    def diag_cb(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.diag = dict(d, _t=time.time())

    def joint_cb(self, m):
        now = time.time()
        if now - self.joints.get('_t', 0) < 0.2:      # 50 Hz -> 5 Hz
            return
        with self.lock:
            self.joints = {'name': list(m.name), 'pos': [round(p, 3) for p in m.position],
                           'vel': [round(v, 3) for v in m.velocity], '_t': now}

    def history_tick(self):
        with self.lock:
            st, d = self.status, self.diag
            if not st.get('connected'):
                return
            mt, dt = d.get('motor_temp') or [], d.get('driver_temp') or []
        self._bat_tick += 1
        if self._bat_tick % 10 == 1:                     # 10 s: 배터리 장기 기록 (디스크)
            with self.lock:
                self.batlog.add(st.get('battery'), st.get('battery_current'), st.get('charging'),
                                st.get('motion_state'), max(mt) if mt else None, max(dt) if dt else None)
        with self.lock:
            self.history.append([round(time.time()), st.get('battery'), st.get('battery_current'),
                                 max(mt) if mt else None, max(dt) if dt else None])

    def sys_tick(self):
        try:
            info = self.sys.collect()
        except Exception as e:
            info = {'error': str(e)}
        info['settings'] = cfg.load()
        info['ros_nodes'] = sorted(f"{ns.rstrip('/')}/{n}" for n, ns in self.get_node_names_and_namespaces())
        with self.lock:
            self.sysinfo = info

    def restart_bridge(self):
        self.event('브리지 재시작 요청 (관리 화면도 잠시 끊겼다가 다시 연결됨)', True)
        # 이 웹 서버도 같은 서비스 안에서 돌기 때문에 분리된 프로세스로 재시작을 건다
        subprocess.Popen(['sudo', '-n', 'systemctl', 'restart', 'bpx-bridge'], start_new_session=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def batt_cb(self, m):
        with self.lock:
            self.battery = {'percent': round(m.percentage * 100), 'current': round(m.current, 2)}

    def odom_cb(self, m):
        t, p = m.twist.twist, m.pose.pose
        with self.lock:
            self.odom = {'vx': round(t.linear.x, 2), 'vy': round(t.linear.y, 2), 'wz': round(t.angular.z, 2)}
            if self.test:
                self.test.add_odom(p.position.x, p.position.y, yaw_of(p.orientation),
                                   t.linear.x, t.linear.y, t.angular.z)

    def imu_cb(self, m):
        now = time.time()
        if now - self.imu.get('_t', 0) < 0.1:          # 10 Hz
            return
        q, a = m.orientation, m.linear_acceleration
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        with self.lock:
            self.imu = {'roll': round(roll, 2), 'pitch': round(pitch, 2), 'yaw': round(math.degrees(yaw_of(q)), 1),
                        'ax': round(a.x, 2), 'ay': round(a.y, 2), 'az': round(a.z, 2), '_t': now}

    # ---------------- 점검 (주행 점검 / 거리 / 전력) ----------------
    def test_begin(self, name):
        if name not in DriveTest.PRESETS:
            raise ValueError(f"알 수 없는 점검: {name}")
        with self.lock:
            self.test = DriveTest(name)
        return self.event(f"점검 시작: {DriveTest.PRESETS[name][0]}")

    def test_end(self, actual_m=None):
        with self.lock:
            t, self.test = self.test, None
        if not t:
            return None
        res = t.analyze(float(actual_m) if actual_m else None)
        res['time'] = time.strftime('%H:%M:%S')
        with self.lock:
            self.test_results = (self.test_results + [res])[-30:]
        ok = res.get('direction_ok')
        self.event(f"점검 결과: {res['label']} " + (res.get('error') or (
            ('방향 OK' if ok else '방향 확인 필요') if ok is not None else f"{res.get('dist_m', 0)} m")), ok if ok is not None else True)
        return res

    def map_cb(self, m):
        with self.lock:
            self.map_msg = m
            self.map_version += 1
        self.event(f"지도 수신 {m.info.width}x{m.info.height} ({m.info.resolution} m/칸)")

    def pose_tick(self):
        st = self.sessions.status()
        cands = [('map', 'base_link'), ('odom', 'base_link')]
        if st and st['alive'] and st['kind'] == 'localization' and st['info'].get('source') == 'bag':
            cands.insert(0, ('map', 'bag_base_link'))     # 시험 bag 위치추정은 브리지와 겹치지 않는 별도 프레임
        for frame, child in cands:
            try:
                tr = self.tf.lookup_transform(frame, child, rclpy.time.Time())
            except Exception:
                continue
            x, y = tr.transform.translation.x, tr.transform.translation.y
            yaw = yaw_of(tr.transform.rotation)
            with self.lock:
                if self.path_frame != frame:
                    self.path.clear()
                    self.path_frame = frame
                if not self.path or math.hypot(x - self.path[-1][0], y - self.path[-1][1]) > 0.05:
                    self.path.append((round(x, 3), round(y, 3)))
                self.pose = (frame, x, y, yaw)
            return
        with self.lock:
            self.pose = None

    # ---------------- 조이스틱 ----------------
    def set_teleop(self, vx, vy, wz):
        with self.lock:
            self.teleop = (clamp(vx, self.max_v[0]), clamp(vy, self.max_v[1]), clamp(wz, self.max_v[2]))
            self.teleop_t = time.monotonic()
            if self.test and any(abs(c) > 1e-3 for c in self.teleop):
                self.test.mark_cmd()

    def teleop_tick(self):
        with self.lock:
            fresh = time.monotonic() - self.teleop_t < TELEOP_TIMEOUT_S
            v = self.teleop if fresh else (0.0, 0.0, 0.0)
            send = fresh or self.teleop_active
            self.teleop_active = fresh and any(abs(c) > 1e-3 for c in v)
        if send:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = v
            self.cmd_pub.publish(t)

    # ---------------- 명령 ----------------
    async def call(self, client, req, label):
        if not client.wait_for_service(timeout_sec=0.5):
            return self.event(f"{label}: 브리지 서비스 없음 (브리지 실행 확인)", False)
        fut = client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.05)
        if not fut.done():
            return self.event(f"{label}: 응답 없음", False)
        res = fut.result()
        return self.event(f"{label}: {res.message}", res.success)

    async def action(self, name):
        return await self.call(self.trig[name], Trigger.Request(),
                               {'stand': '일어서기', 'sit': '앉기', 'damping': '힘 풀기', 'zero': '관절 영점',
                                'walk': '걷기', 'run': '달리기'}[name])

    async def set_motion(self, enable):
        self.set_teleop(0, 0, 0)
        return await self.call(self.motion_cli, SetBool.Request(data=bool(enable)), '조종 모드')

    def send_goal(self, x, y, yaw):
        if not self.nav.wait_for_server(timeout_sec=0.5):
            return self.event('목표 주행: Nav2 가 실행 중이 아님', False)
        with self.lock:
            if not self.pose or self.pose[0] != 'map':
                return self.event('목표 주행: 지도 좌표(map)가 없음 (위치추정 필요)', False)
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        with self.lock:
            self.goal = (x, y, yaw)
            self.goal_state = '전송'
        self.nav.send_goal_async(g).add_done_callback(self._goal_accepted)
        return self.event(f"목표 주행 전송 ({x:.2f}, {y:.2f})", True)

    def _goal_accepted(self, fut):
        h = fut.result()
        if not h.accepted:
            with self.lock:
                self.goal_state = '거부됨'
            return self.event('목표 주행: Nav2 가 거부', False)
        with self.lock:
            self.goal_handle, self.goal_state = h, '진행 중'
        h.get_result_async().add_done_callback(self._goal_done)

    def _goal_done(self, fut):
        st = GOAL_STATUS.get(fut.result().status, str(fut.result().status))
        with self.lock:
            self.goal_state, self.goal_handle = st, None
        self.event(f"목표 주행 종료: {st}")

    def cancel_goal(self):
        with self.lock:
            h = self.goal_handle
        if h:
            h.cancel_goal_async()
            return self.event('목표 주행 취소 요청', True)
        return self.event('취소할 목표 없음', False)

    # ---------------- 화면용 ----------------
    def event(self, text, ok=None):
        self.events.append({'t': time.strftime('%H:%M:%S'), 'text': text, 'ok': ok})
        self.get_logger().info(text)
        return ok

    def state(self):
        with self.lock:
            st = dict(self.status)
            age = time.time() - st.pop('_t', 0) if st else None
            return {
                'type': 'state',
                'bridge_alive': age is not None and age < 3.0,
                'status': st, 'battery': self.battery, 'odom': self.odom,
                'pose': self.pose, 'path': list(self.path)[-1500:], 'path_frame': self.path_frame,
                'goal': self.goal, 'goal_state': self.goal_state,
                'teleop_active': self.teleop_active, 'max_v': self.max_v,
                'map_version': self.map_version,
                'events': list(self.events)[-15:],
                'diag': {k: v for k, v in self.diag.items() if k != '_t'},
                'joints': {k: v for k, v in self.joints.items() if k != '_t'},
                'session': self.sessions.status(),
                'active_map': self.maps.active(),
                'imu': {k: v for k, v in self.imu.items() if k != '_t'},
                'test_active': self.test.name if self.test else None,
                'test_results': self.test_results[-12:],
            }

    def map_payload(self):
        with self.lock:
            m = self.map_msg
            ver = self.map_version
            cached = self._map_cache if getattr(self, '_map_cache_ver', -1) == ver else None
        if m is None:
            return None
        if cached:
            return cached
        i = m.info
        g = np.array(m.data, dtype=np.int8).reshape(i.height, i.width)
        k = max(1, math.ceil(math.sqrt(i.width * i.height / MAP_MAX_CELLS)))
        if k > 1:   # 큰 지도는 줄여서 보낸다 (장애물 > 빈 공간 > 미확인 우선, 얇은 벽 보존)
            g = pool_grid(g, k, 100, 0, -1)
        h, w = g.shape
        payload = {'type': 'map', 'version': ver, 'w': w, 'h': h, 'res': i.resolution * k, 'scale': k,
                   'ox': i.origin.position.x, 'oy': i.origin.position.y, 'oyaw': yaw_of(i.origin.orientation),
                   'data': base64.b64encode(g.astype(np.uint8).tobytes()).decode()}   # -1 -> 255 미지
        with self.lock:
            self._map_cache, self._map_cache_ver = payload, ver
        return payload

    # ---------------- 매핑 실시간 미리보기 ----------------
    def cloud_cb(self, m):
        """FAST-LIO /cloud_registered (camera_init 좌표) 를 0.2 m 격자 윗면도로 누적. 새 칸만 화면에 보낸다."""
        if not self.sessions.running('mapping'):
            return
        self._cloud_n = getattr(self, '_cloud_n', 0) + 1
        if self._cloud_n % 2:                       # 10 Hz -> 5 Hz 처리
            return
        f = {x.name: x.offset for x in m.fields}
        if not {'x', 'y', 'z'} <= set(f):
            return
        n = m.width * m.height
        if n == 0:
            return
        pts = np.frombuffer(bytes(m.data), dtype=np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                            'offsets': [f['x'], f['y'], f['z']], 'itemsize': m.point_step}), count=n)
        xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=1).astype(np.float64)
        R = self._preview_level()
        if R is not None:
            xyz = xyz @ R.T
        # 바닥·천장을 빼고 라이다 높이 근처만 (중력 보정 후 라이다 기준 -0.3 ~ +1.8 m) -> 벽·장애물 윤곽
        z0 = self.lio_z if self.lio_z is not None else 0.0
        sel = (xyz[:, 2] > z0 - PREVIEW_Z[0]) & (xyz[:, 2] < z0 + PREVIEW_Z[1])
        xyz = xyz[sel]
        if not len(xyz):
            return
        ix = np.floor(xyz[:, 0] / PREVIEW_RES).astype(np.int32)
        iy = np.floor(xyz[:, 1] / PREVIEW_RES).astype(np.int32)
        keys = np.unique(np.stack([ix, iy], axis=1), axis=0)
        with self.lock:
            for a, b in keys.tolist():
                if (a, b) not in self.preview_set:
                    self.preview_set.add((a, b))
                    self.preview_cells.append((a, b))

    def lio_odom_cb(self, m):
        if not self.sessions.running('mapping'):
            return
        p = m.pose.pose.position
        v = np.array([p.x, p.y, p.z])
        R = self._preview_level()
        yaw = yaw_of(m.pose.pose.orientation)
        if R is not None:
            v = R @ v
            fwd = R @ np.array([math.cos(yaw), math.sin(yaw), 0.0])
            yaw = math.atan2(fwd[1], fwd[0])
        x, y = float(v[0]), float(v[1])
        with self.lock:
            self.lio_z = float(v[2])
            self.lio_pose = (x, y, yaw)
            if not self.lio_path or math.hypot(x - self.lio_path[-1][0], y - self.lio_path[-1][1]) > 0.1:
                self.lio_path.append((round(x, 2), round(y, 2)))

    def _preview_level(self):
        """매핑 시작 때 잰 중력(gravity.json)으로 수평 보정 회전. 파일이 생기기 전엔 None."""
        if self.preview_R is not None:
            return self.preview_R
        st = self.sessions.status()
        if not st or st['kind'] != 'mapping':
            return None
        try:
            g = json.load(open(self.maps.path(st['info']['map'], 'gravity.json')))['gravity_imu']
        except (OSError, ValueError, KeyError):
            return None
        u = np.asarray(g, float) / np.linalg.norm(g)
        vv = np.cross(u, [0.0, 0.0, 1.0]); sn, c = np.linalg.norm(vv), u[2]
        if sn < 1e-9:
            self.preview_R = np.eye(3)
        else:
            k = np.array([[0, -vv[2], vv[1]], [vv[2], 0, -vv[0]], [-vv[1], vv[0], 0]])
            self.preview_R = np.eye(3) + k + k @ k * ((1 - c) / sn ** 2)
        return self.preview_R

    def reset_preview(self):
        with self.lock:
            self.preview_set, self.preview_cells = set(), []
            self.lio_path, self.lio_pose, self.lio_z, self.preview_R = [], None, None, None
            self.preview_gen = getattr(self, 'preview_gen', 0) + 1

    # ---------------- 매핑 / 위치추정 세션 ----------------
    def _source_args(self, source, kind):
        if source == 'bag':
            bag = self.test_bag_custom if kind == 'mapping' else self.test_bag_pc2
            if not os.path.isdir(bag):
                raise ValueError(f"시험용 bag 없음: {bag}")
            return ['source:=bag', f'bag:={bag}', f'rate:={self.test_bag_rate}']
        return ['source:=live'] + cfg.lidar_launch_args()

    def mapping_start(self, name, source):
        if self.maps.path(name) and os.path.exists(self.maps.path(name)):
            raise ValueError(f"같은 이름의 지도가 이미 있음: {name}")
        cmd = ['ros2', 'launch', 'bpx_ros', 'mapping_session.launch.py', f'map_name:={name}'] + self._source_args(source, 'mapping')
        self.reset_preview()
        self.sessions.start('mapping', cmd, {'map': name, 'source': source})
        os.makedirs(self.maps.path(name), exist_ok=True)
        self.maps.write_meta(name, created=time.strftime('%Y-%m-%d %H:%M'),
                             source='시험 bag' if source == 'bag' else f"라이다 {cfg.load()['lidar']['ip']}")
        return self.event(f"매핑 시작: {name} ({'시험 bag' if source == 'bag' else '라이다'})", True)

    async def mapping_save(self):
        st = self.sessions.status()
        if not st or st['kind'] != 'mapping' or not st['alive']:
            return self.event('저장할 매핑 세션이 없음', False)
        name = st['info']['map']
        ok = await self.call(self.map_save_cli, Trigger.Request(), '지도 저장')
        if not ok:
            return False
        await asyncio.get_event_loop().run_in_executor(None, self.sessions.stop)
        self.event(f"매핑 종료, 지도 처리 시작: {name}")
        self.maps.process_async(name, done=lambda n: self.event(
            f"지도 처리 {'완료' if self.maps.jobs[n]['state'] == 'done' else '실패: ' + self.maps.jobs[n]['error']}: {n}",
            self.maps.jobs[n]['state'] == 'done'))
        return True

    async def mapping_cancel(self):
        st = self.sessions.status()
        await asyncio.get_event_loop().run_in_executor(None, self.sessions.stop)
        if st and st['kind'] == 'mapping':
            name = st['info']['map']
            if not os.path.exists(self.maps.path(name, 'map.pcd')):
                shutil.rmtree(self.maps.path(name), ignore_errors=True)
            return self.event(f"매핑 취소 (저장 안 함): {name}", True)
        return self.event('세션 종료', True)

    def loc_start(self, name, source, nav):
        info = self.maps.info(name)
        if not info['ready']:
            raise ValueError('처리가 끝난 지도만 쓸 수 있음 (수평 보정·2D 지도)')
        cmd = ['ros2', 'launch', 'bpx_ros', 'localization_session.launch.py', f'map_dir:={self.maps.path(name)}',
               f"nav:={'true' if nav else 'false'}"] + self._source_args(source, 'loc')
        self.sessions.start('localization', cmd, {'map': name, 'source': source, 'nav': bool(nav)})
        return self.event(f"위치추정 시작: {name}{' + Nav2' if nav else ''} ({'시험 bag' if source == 'bag' else '라이다'})", True)

    async def session_stop(self):
        st = self.sessions.status()
        await asyncio.get_event_loop().run_in_executor(None, self.sessions.stop)
        return self.event(f"{'매핑' if st and st['kind'] == 'mapping' else '위치추정'} 세션 종료", True)

    async def set_limits(self, vx, vy, wz, timeout):
        def dparam(name, v):
            return ParamMsg(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))
        if not self.param_cli.wait_for_service(timeout_sec=0.5):
            return self.event('속도 상한: 브리지 없음', False)
        req = SetParameters.Request(parameters=[dparam('max_vx', vx), dparam('max_vy', vy), dparam('max_wz', wz),
                                                dparam('cmd_timeout', timeout)])
        fut = self.param_cli.call_async(req)
        for _ in range(100):
            if fut.done():
                break
            await asyncio.sleep(0.05)
        if not fut.done():
            return self.event('속도 상한: 응답 없음', False)
        bad = [r.reason for r in fut.result().results if not r.successful]
        if bad:
            return self.event('속도 상한 거부: ' + bad[0], False)
        return self.event(f"속도 상한 변경: 전후 {vx} · 좌우 {vy} m/s · 회전 {wz} rad/s · 타임아웃 {timeout} s"
                          " (브리지를 다시 켜면 기본값으로 돌아감)", True)

    def nav_test_start(self):
        """Nav2 시험 (라이다·지도 없이): 로봇 주변 빈 지도 + map->odom 고정 + 실제 다리 odom."""
        with self.lock:
            pose = self.pose
        if not pose:
            raise ValueError('로봇 위치(odom)가 없음 (로봇 연결 확인)')
        d = os.path.expanduser('~/.ros/bpx_web_sessions')
        os.makedirs(d, exist_ok=True)
        size, res = 30.0, 0.05                       # 로봇 주변 30 x 30 m 빈 지도
        n = int(size / res)
        with open(os.path.join(d, 'navtest_map.pgm'), 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (n, n) + bytes([254]) * (n * n))
        with open(os.path.join(d, 'navtest_map.yaml'), 'w') as f:
            f.write(f"image: navtest_map.pgm\nmode: trinary\nresolution: {res}\n"
                    f"origin: [{pose[1] - size / 2:.3f}, {pose[2] - size / 2:.3f}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        cmd = ['ros2', 'launch', 'bpx_ros', 'nav.launch.py', f"map:={os.path.join(d, 'navtest_map.yaml')}"]
        self.sessions.start('nav_test', cmd, {'map': '(빈 지도)', 'source': 'odom', 'nav': True})
        return self.event('Nav2 시험 시작 (라이다 없이, 다리 odom 만) · 지도에서 "목표 지정"', True)

    def init_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.07
        msg.pose.covariance = cov
        self.initpose_pub.publish(msg)
        return self.event(f"초기 위치 지정 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)", True)

    def mapping_payload(self, sent):
        with self.lock:
            cells = self.preview_cells[sent:]
            return {'type': 'mapping', 'gen': self.preview_gen, 'res': PREVIEW_RES, 'from': sent, 'cells': cells,
                    'total': len(self.preview_cells), 'path': self.lio_path[-3000:], 'pose': self.lio_pose}, len(self.preview_cells)


MAP_COMMANDS = ('mapping_start', 'mapping_save', 'mapping_cancel', 'loc_start', 'session_stop',
                'map_activate', 'map_rename', 'map_delete', 'map_process', 'init_pose',
                'test_begin', 'test_end', 'test_cancel', 'set_limits', 'save_settings', 'nav_test_start')


async def run_map_command(node: WebNode, kind, msg):
    """매핑·지도 관리 명령. 잘못된 요청은 화면 기록에 실패로 남긴다."""
    try:
        if kind == 'mapping_start':
            node.mapping_start(msg.get('name', ''), msg.get('source', 'live'))
        elif kind == 'mapping_save':
            await node.mapping_save()
        elif kind == 'mapping_cancel':
            await node.mapping_cancel()
        elif kind == 'loc_start':
            node.loc_start(msg.get('name') or node.maps.active() or '', msg.get('source', 'live'), msg.get('nav', False))
        elif kind == 'session_stop':
            await node.session_stop()
        elif kind == 'map_activate':
            node.maps.set_active(msg['name'])
            node.event(f"활성 지도: {msg['name']}", True)
        elif kind == 'map_rename':
            st = node.sessions.status()
            if st and st['alive'] and st['info'].get('map') == msg['old']:
                raise ValueError('이 지도를 쓰는 세션이 실행 중')
            node.maps.rename(msg['old'], msg['new'])
            node.event(f"지도 이름 변경: {msg['old']} → {msg['new']}", True)
        elif kind == 'map_delete':
            st = node.sessions.status()
            node.maps.delete(msg['name'], busy=bool(st and st['alive'] and st['info'].get('map') == msg['name']))
            node.event(f"지도 삭제: {msg['name']}", True)
        elif kind == 'map_process':
            node.maps.process_async(msg['name'], done=lambda n: node.event(
                f"지도 처리 {'완료' if node.maps.jobs[n]['state'] == 'done' else '실패'}: {n}",
                node.maps.jobs[n]['state'] == 'done'))
            node.event(f"지도 다시 처리: {msg['name']}")
        elif kind == 'init_pose':
            node.init_pose(msg['x'], msg['y'], msg.get('yaw', 0.0))
        elif kind == 'test_begin':
            node.test_begin(msg.get('name', ''))
        elif kind == 'test_end':
            node.test_end(msg.get('actual_m'))
        elif kind == 'set_limits':
            await node.set_limits(msg['vx'], msg['vy'], msg['wz'], msg.get('timeout', 0.5))
        elif kind == 'save_settings':
            cfg.save(msg.get('settings', {}))
            node.event('설정 저장 (다음 매핑·위치추정·지도 처리부터 적용)', True)
        elif kind == 'nav_test_start':
            node.nav_test_start()
        elif kind == 'test_cancel':
            with node.lock:
                node.test = None
            node.event('점검 취소')
    except (ValueError, RuntimeError, KeyError, OSError) as e:
        node.event(f"{kind}: {e}", False)


def make_app(node: WebNode) -> FastAPI:
    app = FastAPI(title='BPX 관리자')
    types = {'.html': 'text/html; charset=utf-8', '.js': 'application/javascript; charset=utf-8',
             '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml'}

    def static(name):
        path = os.path.join(STATIC_DIR, os.path.basename(name))
        if not os.path.isfile(path):
            return Response(status_code=404)
        with open(path, 'rb') as f:
            return Response(f.read(), media_type=types.get(os.path.splitext(path)[1], 'application/octet-stream'),
                            headers={'Cache-Control': 'no-cache'})

    @app.get('/')
    def index():
        return static('index.html')

    @app.get('/static/{name}')
    def static_file(name: str):
        return static(name)

    @app.get('/api/state')
    def api_state():
        return node.state()

    @app.get('/api/maps')
    def api_maps():
        return node.maps.list()

    @app.get('/api/maps/{name}/preview.png')
    def api_preview(name: str):
        try:
            png = node.maps.preview_png(name)
        except ValueError:
            png = None
        if png is None:
            return Response(status_code=404)
        return Response(png, media_type='image/png', headers={'Cache-Control': 'no-cache'})

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        node.event('관리 화면 접속')
        sent_map = -1
        tick = 0
        sent_cells, sent_gen = 0, -1

        async def sender():
            nonlocal sent_map, tick, sent_cells, sent_gen
            while True:
                st = node.state()
                if st['map_version'] != sent_map:
                    mp = node.map_payload()
                    if mp:
                        await sock.send_text(json.dumps(mp))
                    sent_map = st['map_version']
                await sock.send_text(json.dumps(st))
                tick += 1
                if tick % 10 == 1:                    # 2 s: 시스템 상태
                    with node.lock:
                        info = dict(node.sysinfo)
                    await sock.send_text(json.dumps({'type': 'system', **info}))
                if tick % 10 == 2:                    # 2 s: 지도 목록
                    lst = await asyncio.get_event_loop().run_in_executor(None, node.maps.list)
                    await sock.send_text(json.dumps({'type': 'maps', 'maps': lst}))
                if tick % 5 == 0 and (node.sessions.running('mapping') or sent_gen != node.preview_gen):
                    if sent_gen != node.preview_gen:  # 새 매핑 세션 -> 처음부터 다시
                        sent_cells, sent_gen = 0, node.preview_gen
                    mp, sent_cells = node.mapping_payload(sent_cells)   # 1 s: 매핑 미리보기 (새 칸만)
                    await sock.send_text(json.dumps(mp))
                if tick % 50 == 3:                    # 10 s: 배터리 장기 기록·추정
                    with node.lock:
                        bp = node.batlog.payload()
                    await sock.send_text(json.dumps(bp))
                if tick % 25 == 1:                    # 5 s: 30 분 추이
                    with node.lock:
                        hist = list(node.history)
                    await sock.send_text(json.dumps({'type': 'history', 'rows': hist}))
                await asyncio.sleep(0.2)

        task = asyncio.ensure_future(sender())
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get('type')
                if kind == 'teleop':
                    node.set_teleop(msg.get('vx', 0), msg.get('vy', 0), msg.get('wz', 0))
                elif kind == 'restart_bridge':
                    node.restart_bridge()
                elif kind in MAP_COMMANDS:
                    await run_map_command(node, kind, msg)
                elif kind == 'action' and msg.get('name') in ('stand', 'sit', 'damping', 'zero', 'walk', 'run'):
                    await node.action(msg['name'])
                elif kind == 'motion':
                    await node.set_motion(msg.get('enable', False))
                elif kind == 'goal':
                    node.send_goal(msg['x'], msg['y'], msg.get('yaw', 0.0))
                elif kind == 'cancel_goal':
                    node.cancel_goal()
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()
            node.set_teleop(0, 0, 0)          # 화면이 닫히면 즉시 정지
            node.event('관리 화면 접속 끊김')

    return app


def main():
    rclpy.init()
    node = WebNode()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()
    node.get_logger().info(f"관리자 웹: http://0.0.0.0:{node.port}")
    try:
        # Ubuntu 22.04 의 websockets 9.1 은 Python 3.10 과 호환 안 됨 -> wsproto 사용
        uvicorn.run(make_app(node), host='0.0.0.0', port=node.port, log_level='warning', ws='wsproto')
    finally:
        node.set_teleop(0, 0, 0)
        node.teleop_tick()
        ex.shutdown()
        spin_thread.join(timeout=2.0)   # spin 스레드가 살아 있는 채 종료하면 C++ terminate(-6)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
