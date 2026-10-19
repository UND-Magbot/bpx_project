#!/usr/bin/env python3
"""BPX SDK <-> ROS 2 브리지 (Zishen cmd_vel_bridge.py 구조를 BPX SDK로 이식)

  /cmd_vel (Twist)            -> MotionLevelControl.setVelocity(x, y, yaw)  (cmd_rate Hz, 타임아웃 시 0)
  LegOdom                     -> /odom/mc_odom (Odometry) + TF odom->base_link
  관절 / IMU / 배터리          -> /joint_states, /imu/data, /battery_state
  상태                         -> /bpx/status (String JSON)
  서비스 ~/stand ~/sit ~/damping ~/zero(관절 영점, 엎드림에서만) (std_srvs/Trigger)
  서비스 ~/set_motion (std_srvs/SetBool)  실행 중 조종 모드 전환. 끄면 즉시 속도 0 + 플래그 해제 (관리자 웹이 사용)
  토픽 /bpx/motion (String: stand|sit|damping)  <- 관제 브리지 연동 훅 (Zishen /zishen/motion 과 같은 역할)

SDK 소유는 이 노드 하나로 유지한다. 앱/Mind+ 와 동시 조종 금지.
allow_motion:=false 이면 상태 읽기 전용(명령을 한 번도 보내지 않음) -> 첫 실기 연결 때 사용.
"""
import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from bpx_ros.backend import (CM_NAVIGATOR, CM_REMOTE, JOINT_NAMES, MS_LYING, MS_MOTION,
                             make_backend)

STAND_TIMEOUT_S = 15.0   # Mind+ 확장과 같은 값
SIT_TIMEOUT_S = 15.0
RESEND_S = 0.2           # 서기/앉기 요청 재전송 간격 (Mind+ INTERVAL)
ZERO_SETTLE_S = 1.0      # 영점 설정 후 서기까지 대기 (SDK 예제·Mind+ reset_joints 와 같음)


def clamp(v, lim):
    return max(-lim, min(lim, v))


def finite(seq, n):
    try:
        return seq is not None and len(seq) >= n and all(math.isfinite(float(x)) for x in seq[:n])
    except (TypeError, ValueError):
        return False


class BpxBridge(Node):
    # mode: idle(명령 안 보냄) / standing(서기 요청 중) / ready(속도 스트리밍) / sitting(앉기 요청 중)
    def __init__(self):
        super().__init__("bpx_bridge")
        p = self.declare_parameter
        self.backend_kind = p("backend", "sdk").value
        robot_ip = p("robot_ip", "10.21.20.1").value
        state_port = p("state_port", 9873).value
        state_rate = p("state_rate", 50).value
        cmd_rate = p("cmd_rate", 50).value
        publish_rate = p("publish_rate", 50).value
        self.allow_motion = p("allow_motion", False).value
        self.auto_stand = p("auto_stand", False).value
        self.sit_on_exit = p("sit_on_exit", True).value
        self.yield_to_remote = p("yield_to_remote", True).value
        self.max_vx = p("max_vx", 0.4).value
        self.max_vy = p("max_vy", 0.25).value
        self.max_wz = p("max_wz", 1.0).value
        self.cmd_timeout = p("cmd_timeout", 0.5).value
        cmd_topic = p("cmd_topic", "/cmd_vel").value
        odom_topic = p("odom_topic", "/odom/mc_odom").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.imu_frame = p("imu_frame", "base_link").value
        self.publish_tf = p("publish_tf", True).value

        self.m = make_backend(self.backend_kind, robot_ip, state_port, state_rate, cmd_rate)
        self.get_logger().info(
            f"backend={self.backend_kind} robot={robot_ip} state_udp={state_port} "
            f"allow_motion={self.allow_motion}")

        self.mode = "idle"
        self.mode_t = 0.0
        self.last_req = 0.0
        self.connected = False
        self.last_connect_try = 0.0
        self.ever_connected = False      # 한 번 붙은 뒤에는 SDK 자동 재연결에 맡김
        self.motion_state = None
        self.zero_until = 0.0
        self.control_mode = None
        self.nav_seen = False            # ready 진입 후 Navigator(SDK 조종)를 확인했는지
        self.nav_warned = False
        self.vx = self.vy = self.wz = 0.0
        self.last_cmd = 0.0

        # 발행
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos_profile_sensor_data)
        self.batt_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.status_pub = self.create_publisher(String, "/bpx/status", 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        # 구독 / 서비스
        self.create_subscription(Twist, cmd_topic, self.cmd_cb, 10)
        self.create_subscription(String, "/bpx/motion", self.motion_cb, 10)
        self.create_service(Trigger, "~/stand", lambda rq, rs: self.srv(rs, "stand"))
        self.create_service(Trigger, "~/sit", lambda rq, rs: self.srv(rs, "sit"))
        self.create_service(Trigger, "~/damping", lambda rq, rs: self.srv(rs, "damping"))
        self.create_service(Trigger, "~/zero", lambda rq, rs: self.srv(rs, "zero"))   # 관절 영점
        self.create_service(SetBool, "~/set_motion", self.set_motion_cb)

        # 타이머 (단일 스레드 executor -> SDK 호출이 겹치지 않음)
        self.create_timer(1.0 / cmd_rate, self.stream)
        self.create_timer(1.0 / publish_rate, self.publish_state)
        self.create_timer(0.1, self.supervise)
        self.status_tick = 0
        self.create_timer(0.2, self.publish_slow)    # /bpx/status 5 Hz (관리자 웹 반응), /battery_state 는 1 Hz
        self.get_logger().info(f"구독: {cmd_topic}  발행: {odom_topic}, /joint_states, /imu/data")

    # ---------------- 명령 입력 ----------------
    def cmd_cb(self, msg):
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in vals):
            return
        self.vx = clamp(vals[0], self.max_vx)
        self.vy = clamp(vals[1], self.max_vy)
        self.wz = clamp(vals[2], self.max_wz)
        self.last_cmd = time.monotonic()

    def motion_cb(self, msg):
        what = (msg.data or "").strip().lower()
        if what == "zero":   # 영점은 자세 확인이 필요해서 관제 토픽으로는 받지 않고 ~/zero 서비스로만
            self.get_logger().warn("/bpx/motion: zero 는 ~/zero 서비스로만 허용")
            return
        ok, text = self.request(what)
        (self.get_logger().info if ok else self.get_logger().warn)(f"/bpx/motion: {text}")

    def set_motion_cb(self, req, res):
        self.set_motion(bool(req.data))
        res.success, res.message = True, f"allow_motion={self.allow_motion}"
        return res

    def set_motion(self, enable):
        if enable == self.allow_motion:
            return
        if not enable:
            # 조종 모드 해제: 진행 중인 서기/주행/앉기 요청을 멈추고 속도 0. 서 있던 로봇은 펌웨어가 계속 세워 둔다
            if self.connected and self.mode != "idle":
                self.zero_and_flag_off()
            self.vx = self.vy = self.wz = 0.0
            self.set_mode("idle")
        self.allow_motion = enable
        self.get_logger().warn(f"조종 모드 {'켜짐 (명령 허용)' if enable else '꺼짐 (읽기 전용)'}")

    def srv(self, res, what):
        res.success, res.message = self.request(what)
        return res

    def request(self, what):
        if what not in ("stand", "sit", "damping", "zero"):
            return False, f"알 수 없는 명령 '{what}' (stand|sit|damping|zero)"
        if not self.allow_motion:
            return False, "allow_motion=false (읽기 전용) -> 명령 거부"
        if not self.connected:
            return False, "로봇 미연결"
        if what == "zero":
            # 관절 영점 (앱 Reset Joints 와 같음). 네 다리 발·정강이·무릎이 바닥에 닿은 엎드림에서만
            if self.motion_state != MS_LYING or self.mode != "idle":
                return False, f"엎드림·대기 상태에서만 가능 (motion_state={self.motion_state}, mode={self.mode})"
            self.m.setZeroPositionsFlag()
            self.zero_until = time.monotonic() + ZERO_SETTLE_S
            self.get_logger().warn("관절 영점 설정 전송")
            return True, "관절 영점 설정 전송 (1초 후 일어서기 가능)"
        if what == "stand":
            self.set_mode("standing")
        elif what == "sit":
            self.zero_and_flag_off()
            self.set_mode("sitting")
        else:
            self.zero_and_flag_off()
            self.m.setDamping()          # 소프트웨어 힘풀기 (하드웨어 E-stop 아님)
            self.set_mode("idle")
        return True, f"{what} 요청됨"

    def set_mode(self, mode):
        if mode != self.mode:
            self.get_logger().info(f"mode {self.mode} -> {mode}")
        self.mode, self.mode_t, self.last_req = mode, time.monotonic(), 0.0
        self.nav_seen = self.nav_warned = False

    def zero_and_flag_off(self):
        try:
            self.m.setVelocity(0.0, 0.0, 0.0)
            self.m.setVelocityControlFlag(False)
        except Exception as e:
            self.get_logger().warn(f"정지 명령 실패: {e}")

    # ---------------- 상태 머신 (10Hz) ----------------
    def supervise(self):
        now = time.monotonic()
        try:
            connected = bool(self.m.isConnected())
        except Exception:
            connected = False

        if not connected:
            if self.connected:
                self.get_logger().error("로봇 연결 끊김 -> SDK 자동 재연결 대기")
                # 재연결 후 SDK 는 0 이 아닌 명령을 재전송하지 않는다 -> 서기 절차부터 다시
                if self.mode in ("ready", "standing"):
                    self.mode = "standing"
            self.connected = False
            if now - self.last_connect_try > 2.0 and not self.ever_connected:
                self.last_connect_try = now
                try:
                    self.m.connect()
                except Exception as e:
                    self.get_logger().warn(f"connect 실패: {e}", throttle_duration_sec=10.0)
            return

        if not self.connected:
            self.get_logger().info("로봇 연결됨")
            try:
                ver = getattr(self.m, "getRobotVersion", lambda: None)()
                self.get_logger().info(
                    f"로봇 정보: SN={self.m.getRobotSerialNumber()} model={self.m.getRobotModel()} "
                    f"firmware={ver} sdk={getattr(self.m, '__version__', '?')}")
            except Exception as e:
                self.get_logger().warn(f"로봇 정보 조회 실패: {e}")
            self.ever_connected = True
            self.connected = True
            if self.mode in ("ready", "standing"):
                self.set_mode("standing")
            elif self.auto_stand and self.allow_motion and self.mode == "idle":
                self.set_mode("standing")

        prev_ms = self.motion_state
        try:
            self.motion_state = self.m.getCurrentMotionState()
        except Exception:
            self.motion_state = None
        ms = self.motion_state
        if ms != prev_ms:   # 서기 실패 원인 추적용
            self.get_logger().info(f"motion_state {prev_ms} -> {ms} (mode={self.mode})")
        try:
            self.control_mode = self.m.getControlMode()
        except Exception:
            self.control_mode = None

        if self.mode == "ready" and self.check_takeover(now):
            return

        if self.mode == "standing":
            if ms == MS_MOTION:
                self.m.setVelocityControlFlag(True)
                self.set_mode("ready")
                self.get_logger().info("서기 완료 -> /cmd_vel 수신 시작")
            elif now - self.mode_t > STAND_TIMEOUT_S:
                self.get_logger().error("서기 타임아웃(15s) -> idle")
                self.zero_and_flag_off()
                self.set_mode("idle")
            elif now - self.last_req >= RESEND_S and now >= self.zero_until:
                self.m.setStandUp()
                self.last_req = now
        elif self.mode == "sitting":
            if ms == MS_LYING:              # 3(SitDown)은 전이 중, 0 이어야 완료
                self.get_logger().info("앉기 완료")
                self.set_mode("idle")
            elif now - self.mode_t > SIT_TIMEOUT_S:
                self.get_logger().error("앉기 타임아웃(15s) -> idle")
                self.set_mode("idle")
            elif now - self.last_req >= RESEND_S:
                self.m.setSitDown()
                self.last_req = now
        elif self.mode == "ready" and ms is not None and ms != MS_MOTION:
            # 앱 조작/넘어짐 등으로 Motion 상태를 벗어남 -> 스트리밍 중단
            if now - self.mode_t > 1.0:
                self.get_logger().warn(f"motion state={ms} (Motion 아님) -> idle")
                self.zero_and_flag_off()
                self.set_mode("idle")

    def check_takeover(self, now):
        """앱/리모컨이 조종을 가져갔으면 스트리밍을 멈추고 True.

        우선순위 규칙이 문서에 없으므로 Navigator 를 한 번 확인한 뒤 RemoteControl 로 바뀐 경우만
        탈취로 본다. Navigator 가 끝내 안 보이면 경고만 한다 (실기에서 동작 확인 필요).
        """
        cm = self.control_mode
        if cm == CM_NAVIGATOR:
            self.nav_seen = True
        elif cm == CM_REMOTE and self.nav_seen and self.yield_to_remote:
            self.get_logger().warn("앱/리모컨이 조종을 가져감 (ControlMode=RemoteControl) -> 정지, idle")
            self.zero_and_flag_off()
            self.set_mode("idle")
            return True
        elif not self.nav_seen and not self.nav_warned and now - self.mode_t > 3.0:
            self.get_logger().warn(f"ready 3초 지났는데 ControlMode={cm} (Navigator 아님) -> 실기 동작 확인 필요")
            self.nav_warned = True
        return False

    # ---------------- 속도 스트리밍 (cmd_rate) ----------------
    def stream(self):
        if self.mode != "ready" or not self.connected:
            return
        if time.monotonic() - self.last_cmd > self.cmd_timeout:
            self.vx = self.vy = self.wz = 0.0
        try:
            self.m.setVelocity(self.vx, self.vy, self.wz)
        except Exception as e:
            self.get_logger().warn(f"setVelocity 실패: {e}", throttle_duration_sec=2.0)

    # ---------------- 상태 발행 ----------------
    def publish_state(self):
        if not self.connected:
            return
        now = self.get_clock().now().to_msg()
        m = self.m

        try:
            od = m.getLegOdom()
        except Exception:
            od = None
        if od and finite(od.get("position"), 3) and finite(od.get("orientation"), 4):
            p, q = od["position"], od["orientation"]           # q = x, y, z, w
            if abs(math.sqrt(sum(float(v) ** 2 for v in q[:4])) - 1.0) < 0.1:
                o = Odometry()
                o.header.stamp = now
                o.header.frame_id, o.child_frame_id = self.odom_frame, self.base_frame
                pp, oo = o.pose.pose.position, o.pose.pose.orientation
                pp.x, pp.y, pp.z = (float(v) for v in p[:3])
                oo.x, oo.y, oo.z, oo.w = (float(v) for v in q[:4])
                v, w = od.get("velocity_body"), od.get("angular_velocity")
                if finite(v, 3):
                    o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(x) for x in v[:3])
                if finite(w, 3):
                    o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = (float(x) for x in w[:3])
                self.odom_pub.publish(o)
                if self.tf_br:
                    t = TransformStamped()
                    t.header = o.header
                    t.child_frame_id = self.base_frame
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pp.x, pp.y, pp.z
                    t.transform.rotation = oo
                    self.tf_br.sendTransform(t)

        try:
            pos, vel, tau = m.getJointPosition(), m.getJointVelocity(), m.getJointTorque()
        except Exception:
            pos = None
        if finite(pos, 12):
            js = JointState()
            js.header.stamp = now
            js.name = JOINT_NAMES
            js.position = [float(x) for x in pos[:12]]
            if finite(vel, 12):
                js.velocity = [float(x) for x in vel[:12]]
            # effort: SDK 토크 단위 미확인 -> 확인 전까지 비움 (Robonix primitive 와 동일 판단)
            self.js_pub.publish(js)

        try:
            q, w, a = m.getImuQuat(), m.getImuOmega(), m.getImuAcc()
        except Exception:
            q = None
        if finite(q, 4):
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = now, self.imu_frame
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = (float(x) for x in q[:4])
            if finite(w, 3):
                imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(x) for x in w[:3])
            if finite(a, 3):
                imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(x) for x in a[:3])
            self.imu_pub.publish(imu)

    def publish_slow(self):
        self.status_tick += 1
        publish_battery = self.status_tick % 5 == 1
        st = {"backend": self.backend_kind, "connected": self.connected, "mode": self.mode,
              "motion_state": self.motion_state, "control_mode": self.control_mode,
              "allow_motion": self.allow_motion}
        if self.connected:
            try:
                # 충전기가 꽂혀 있으면 동작을 거부할 수 있음 -> 진단용으로 표시
                st["charger"] = [getattr(self.m, "getChargerIn1", lambda: None)(),
                                 getattr(self.m, "getChargerIn2", lambda: None)()]
                st["max_vel"] = getattr(self.m, "getMaxVelocity", lambda: None)()
                st["sub_gait"] = getattr(self.m, "getSubGait", lambda: None)()
            except Exception:
                pass
            try:
                st["gait"] = self.m.getCurrentGait()
                temps = self.m.getMotorTemperature()
                if finite(temps, 12):
                    st["motor_temp_max"] = round(max(float(t) for t in temps[:12]), 1)   # 단위 미확인(℃ 추정)
            except Exception:
                pass
            try:
                lvl = self.m.getBatteryLevel()
                if lvl is not None and 0 <= int(lvl) <= 100:
                    b = BatteryState()
                    b.header.stamp = self.get_clock().now().to_msg()
                    b.percentage = int(lvl) / 100.0
                    b.present = True
                    cur = self.m.getBatteryCurrent()
                    if cur is not None and math.isfinite(float(cur)):
                        b.current = float(cur)        # 단위/부호 실기 확인 필요
                    if publish_battery:
                        self.batt_pub.publish(b)
                    st["battery"] = int(lvl)
            except Exception:
                pass
        self.status_pub.publish(String(data=json.dumps(st)))

    # ---------------- 종료 ----------------
    def shutdown_robot(self):
        if not (self.allow_motion and self.connected):
            return
        if self.sit_on_exit and self.mode in ("ready", "standing"):
            self.get_logger().info("종료 -> 정지 후 앉기")
            self.zero_and_flag_off()
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    self.m.setSitDown()
                    if self.m.getCurrentMotionState() == MS_LYING:
                        break
                except Exception:
                    break
                time.sleep(RESEND_S)
        else:
            self.zero_and_flag_off()


def main():
    rclpy.init()
    node = BpxBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT -> KeyboardInterrupt, SIGTERM(또는 SIGINT 무시 상속 상태) -> rclpy 가 context 를 내려 ExternalShutdown
        pass
    finally:
        node.shutdown_robot()
        try:
            node.m.disconnect()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
