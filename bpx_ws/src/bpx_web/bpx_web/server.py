#!/usr/bin/env python3
"""BPX 로봇 관리자 웹 서버 (FastAPI + rclpy 한 프로세스)

  ros2 run bpx_web bpx_web --ros-args -p port:=8080
  브라우저: http://<미니PC>:8080

  화면 -> 서버 (WebSocket /ws, JSON)
    {"type":"teleop","vx":..,"vy":..,"wz":..}   조이스틱 (10 Hz 이상으로 계속 보냄, 끊기면 서버가 0 을 보냄)
    {"type":"action","name":"stand|sit|damping"}
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
import threading
import time
from collections import deque

import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener

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
        self.create_subscription(BatteryState, '/battery_state', self.batt_cb, 10)
        self.create_subscription(Odometry, p('odom_topic', '/odom/mc_odom').value, self.odom_cb, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, latched)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_topic', '/cmd_vel').value, 10)
        self.trig = {n: self.create_client(Trigger, f'{bridge}/{n}') for n in ('stand', 'sit', 'damping')}
        self.motion_cli = self.create_client(SetBool, f'{bridge}/set_motion')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.create_timer(0.05, self.teleop_tick)
        self.create_timer(0.2, self.pose_tick)

    # ---------------- 구독 ----------------
    def status_cb(self, m):
        try:
            st = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            prev = self.status
            self.status = dict(st, _t=time.time())
        for k, label in (('connected', '로봇 연결'), ('mode', '브리지 모드'), ('allow_motion', '조종 모드')):
            if prev and prev.get(k) != st.get(k):
                self.event(f"{label}: {prev.get(k)} → {st.get(k)}")

    def batt_cb(self, m):
        with self.lock:
            self.battery = {'percent': round(m.percentage * 100), 'current': round(m.current, 2)}

    def odom_cb(self, m):
        t = m.twist.twist
        with self.lock:
            self.odom = {'vx': round(t.linear.x, 2), 'vy': round(t.linear.y, 2), 'wz': round(t.angular.z, 2)}

    def map_cb(self, m):
        with self.lock:
            self.map_msg = m
            self.map_version += 1
        self.event(f"지도 수신 {m.info.width}x{m.info.height} ({m.info.resolution} m/칸)")

    def pose_tick(self):
        for frame in ('map', 'odom'):
            try:
                tr = self.tf.lookup_transform(frame, 'base_link', rclpy.time.Time())
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
                               {'stand': '일어서기', 'sit': '앉기', 'damping': '힘 풀기'}[name])

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
            }

    def map_payload(self):
        with self.lock:
            m = self.map_msg
            ver = self.map_version
        if m is None:
            return None
        i = m.info
        data = bytes((v + 256) % 256 for v in m.data)   # int8 -> uint8 (-1 = 255 미지)
        return {'type': 'map', 'version': ver, 'w': i.width, 'h': i.height, 'res': i.resolution,
                'ox': i.origin.position.x, 'oy': i.origin.position.y, 'oyaw': yaw_of(i.origin.orientation),
                'data': base64.b64encode(data).decode()}


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

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        node.event('관리 화면 접속')
        sent_map = -1

        async def sender():
            nonlocal sent_map
            while True:
                st = node.state()
                if st['map_version'] != sent_map:
                    mp = node.map_payload()
                    if mp:
                        await sock.send_text(json.dumps(mp))
                    sent_map = st['map_version']
                await sock.send_text(json.dumps(st))
                await asyncio.sleep(0.2)

        task = asyncio.ensure_future(sender())
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get('type')
                if kind == 'teleop':
                    node.set_teleop(msg.get('vx', 0), msg.get('vy', 0), msg.get('wz', 0))
                elif kind == 'action' and msg.get('name') in ('stand', 'sit', 'damping'):
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
    threading.Thread(target=ex.spin, daemon=True).start()
    node.get_logger().info(f"관리자 웹: http://0.0.0.0:{node.port}")
    try:
        # Ubuntu 22.04 의 websockets 9.1 은 Python 3.10 과 호환 안 됨 -> wsproto 사용
        uvicorn.run(make_app(node), host='0.0.0.0', port=node.port, log_level='warning', ws='wsproto')
    finally:
        node.set_teleop(0, 0, 0)
        node.teleop_tick()
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
