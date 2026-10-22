"""bpx_bridge 상태머신 단위시험 (로봇 없이, 가짜 시계 + 명령 기록 Fake 백엔드)

실기에서 재현하기 위험한 경우를 시험한다: 읽기 전용, 서기 타임아웃, 연결 끊김/재연결,
Motion 이탈, /cmd_vel 타임아웃, 속도 제한, 종료 시 앉기.
실행: cd bpx_ws && colcon test --packages-select bpx_ros --event-handlers console_direct+
  또는 python3 -m pytest bpx_ws/src/bpx_ros/test -q
"""
import json
import math

import pytest
import rclpy
from std_msgs.msg import String

from bpx_ros import backend as backend_mod
from bpx_ros import bridge as bridge_mod
from bpx_ros.backend import CM_REMOTE, MS_LYING, MS_PASSIVE, FakeMotion

TICK = 0.1   # supervise 주기


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s


class RecFake(FakeMotion):
    """FakeMotion + SDK 로 나간 명령 기록. stuck=True 면 서기 요청을 무시(안 일어남).
    cm_fixed 를 정하면 속도 플래그와 무관하게 ControlMode 를 그 값으로 고정."""

    RECORDED = ("connect", "setVelocity", "setVelocityControlFlag",
                "setStandUp", "setSitDown", "setDamping", "setZeroPositionsFlag")

    def __init__(self):
        super().__init__()
        self.calls = []
        self.stuck = False
        self.cm_fixed = None

    def __getattribute__(self, name):
        attr = super().__getattribute__(name)
        if name in RecFake.RECORDED:
            calls = super().__getattribute__("calls")

            def wrapped(*a):
                calls.append((name, *a))
                if name == "setStandUp" and self.stuck:
                    return True
                ret = attr(*a)
                if self.cm_fixed is not None:
                    self._control_mode = self.cm_fixed
                return ret
            return wrapped
        return attr

    def names(self):
        return [c[0] for c in self.calls]

    def last(self, name):
        return next((c for c in reversed(self.calls) if c[0] == name), None)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init(args=["--ros-args", "-p", "backend:=fake", "-p", "allow_motion:=true"])
    yield
    rclpy.shutdown()


@pytest.fixture
def clock(monkeypatch):
    c = FakeClock()
    for mod in (bridge_mod, backend_mod):
        monkeypatch.setattr(mod, "time", c)
    return c


@pytest.fixture
def node(clock):
    n = bridge_mod.BpxBridge()
    n.m = RecFake()
    yield n
    n.destroy_node()


def run(node, clock, seconds):
    for _ in range(int(round(seconds / TICK))):
        clock.t += TICK
        node.supervise()


def make_ready(node, clock):
    run(node, clock, 0.2)                      # 연결
    ok, _ = node.request("stand")
    assert ok
    run(node, clock, 2.5)                      # Fake 서기 2초
    assert node.mode == "ready"
    node.m.calls.clear()


# ---------------------------------------------------------------- 읽기 전용
def test_read_only_never_commands(node, clock):
    node.allow_motion = False
    run(node, clock, 1.0)
    for what in ("stand", "sit", "damping"):
        ok, msg = node.request(what)
        assert not ok and "allow_motion" in msg
    node.cmd_cb(_twist(0.3))
    for _ in range(20):
        node.stream()
    run(node, clock, 3.0)
    # connect 외에는 어떤 명령도 로봇으로 나가지 않아야 한다
    assert set(node.m.names()) <= {"connect"}


def test_unknown_and_unconnected_rejected(node, clock):
    ok, _ = node.request("jump")
    assert not ok
    ok, msg = node.request("stand")            # 아직 supervise 전 -> 미연결
    assert not ok and "미연결" in msg


# ---------------------------------------------------------------- 서기
def test_stand_reaches_ready_and_enables_velocity(node, clock):
    run(node, clock, 0.2)
    node.request("stand")
    run(node, clock, 2.5)
    assert node.mode == "ready"
    assert node.m.last("setVelocityControlFlag") == ("setVelocityControlFlag", True)
    assert node.m.names().count("setStandUp") >= 2   # 0.2s 간격 재전송


def test_stand_timeout_goes_idle(node, clock):
    node.m.stuck = True
    run(node, clock, 0.2)
    node.request("stand")
    run(node, clock, 14.0)
    assert node.mode == "standing"
    run(node, clock, 1.5)
    assert node.mode == "idle"
    assert node.m.last("setVelocityControlFlag") == ("setVelocityControlFlag", False)
    assert node.m.last("setVelocity") == ("setVelocity", 0.0, 0.0, 0.0)
    assert ("setVelocityControlFlag", True) not in node.m.calls


# ---------------------------------------------------------------- 연결 끊김 / 재연결
def test_reconnect_restarts_stand_sequence(node, clock):
    make_ready(node, clock)
    node.m._connected = False                  # 연결 끊김
    run(node, clock, 5.0)
    assert node.mode == "standing" and not node.connected
    assert "connect" not in node.m.names()     # 한 번 붙은 뒤에는 SDK 자동 재연결에 맡김
    node.stream()
    assert "setVelocity" not in node.m.names() # 끊긴 동안 속도 스트리밍 안 함

    node.m._connected = True                   # 재연결
    run(node, clock, 0.3)
    assert node.mode == "ready"
    # SDK 는 재연결 후 명령을 재전송하지 않음 -> 속도 플래그를 다시 켜야 한다
    assert node.m.last("setVelocityControlFlag") == ("setVelocityControlFlag", True)


def test_reconnect_while_idle_stays_idle(node, clock):
    run(node, clock, 0.2)
    node.m._connected = False
    run(node, clock, 1.0)
    node.m._connected = True
    run(node, clock, 1.0)
    assert node.mode == "idle"
    assert "setStandUp" not in node.m.names()  # 자동으로 일어서지 않음


# ---------------------------------------------------------------- Motion 이탈
def test_leaving_motion_state_stops_streaming(node, clock):
    make_ready(node, clock)
    clock.t += 2.0                              # ready 진입 후 1초 지남
    node.m._state, node.m._target = MS_PASSIVE, None   # 앱 조작/넘어짐
    run(node, clock, 0.1)
    assert node.mode == "idle"
    assert node.m.last("setVelocityControlFlag") == ("setVelocityControlFlag", False)
    node.m.calls.clear()
    node.cmd_cb(_twist(0.3))
    node.stream()
    assert "setVelocity" not in node.m.names()


# ---------------------------------------------------------------- 보행 전환 / 진단
def test_gait_only_when_ready(node, clock):
    run(node, clock, 0.2)
    ok, msg = node.request("run")
    assert not ok and "ready" in msg
    make_ready(node, clock)
    assert node.request("run")[0] and node.m.getCurrentGait() == 8
    assert node.request("walk")[0] and node.m.getCurrentGait() == 0


def test_gait_rejected_read_only(node, clock):
    make_ready(node, clock)
    node.set_motion(False)
    assert not node.request("run")[0]
    assert node.m.getCurrentGait() == 0


def test_diag_payload(node, clock):
    got = []
    node.diag_pub.publish = lambda msg: got.append(json.loads(msg.data))
    run(node, clock, 0.2)
    node.publish_diag()
    d = got[-1]
    assert len(d["motor_temp"]) == 12 and len(d["driver_temp"]) == 12 and len(d["imu_rpy"]) == 3
    assert d["max_vel"] == [1.5, 0.5, 2.0] and d["firmware"].startswith("1.0.14")


# ---------------------------------------------------------------- 충전 중
def test_stand_rejected_while_charging(node, clock):
    node.m._current = 2.38                      # 실기 충전 중 값
    run(node, clock, 0.2)
    assert node.charging
    ok, msg = node.request("stand")
    assert not ok and "충전" in msg and node.mode == "idle"
    node.m._current = -0.64                     # 충전기 분리
    run(node, clock, 0.2)
    assert not node.charging and node.request("stand")[0]


def test_charging_hysteresis(node, clock):
    node.m._current = 0.6
    run(node, clock, 0.2)
    assert node.charging
    node.m._current = 0.3                       # OFF 기준(0.2)보다 크면 유지
    run(node, clock, 0.2)
    assert node.charging
    node.m._current = 0.1
    run(node, clock, 0.2)
    assert not node.charging


# ---------------------------------------------------------------- 관절 영점
def test_zero_only_when_lying_idle(node, clock):
    run(node, clock, 0.2)
    ok, _ = node.request("zero")
    assert ok and node.m.last("setZeroPositionsFlag") == ("setZeroPositionsFlag",)
    run(node, clock, 1.1)                       # 영점 후 대기
    make_ready(node, clock)                     # 서 있을 때는 거부
    ok, msg = node.request("zero")
    assert not ok and "엎드림" in msg
    assert "setZeroPositionsFlag" not in node.m.names()


def test_zero_rejected_read_only(node, clock):
    node.allow_motion = False
    run(node, clock, 0.2)
    assert not node.request("zero")[0]
    assert "setZeroPositionsFlag" not in node.m.names()


def test_stand_waits_after_zero(node, clock):
    run(node, clock, 0.2)
    node.request("zero")
    node.m.calls.clear()
    node.request("stand")
    run(node, clock, 0.8)                       # 1초 대기 중
    assert "setStandUp" not in node.m.names()
    run(node, clock, 0.4)
    assert "setStandUp" in node.m.names()


def test_zero_not_via_motion_topic(node, clock):
    run(node, clock, 0.2)
    node.motion_cb(String(data="zero"))
    assert "setZeroPositionsFlag" not in node.m.names()


# ---------------------------------------------------------------- 실행 중 조종 모드 전환 (관리자 웹)
def test_enable_motion_at_runtime(node, clock):
    node.allow_motion = False
    run(node, clock, 0.2)
    assert not node.request("stand")[0]
    node.set_motion(True)
    assert node.request("stand")[0]
    run(node, clock, 2.5)
    assert node.mode == "ready"


def test_disable_motion_while_walking_stops(node, clock):
    make_ready(node, clock)
    node.cmd_cb(_twist(0.3))
    node.stream()
    node.m.calls.clear()
    node.set_motion(False)
    assert node.mode == "idle" and not node.allow_motion
    assert node.m.calls[:2] == [("setVelocity", 0.0, 0.0, 0.0), ("setVelocityControlFlag", False)]
    node.m.calls.clear()
    node.cmd_cb(_twist(0.3))
    for _ in range(5):
        node.stream()
    run(node, clock, 1.0)
    assert node.m.calls == []                   # 끈 뒤로는 아무 명령도 안 나감
    assert not node.request("sit")[0]


def test_disable_motion_when_idle_sends_nothing(node, clock):
    run(node, clock, 0.5)
    node.m.calls.clear()
    node.set_motion(False)
    assert node.m.calls == []


# ---------------------------------------------------------------- 조종권 (ControlMode)
def test_remote_takeover_stops_streaming(node, clock):
    make_ready(node, clock)
    run(node, clock, 0.3)                       # Navigator 확인
    assert node.nav_seen
    node.m._control_mode = CM_REMOTE            # 앱/리모컨이 조종을 가져감
    run(node, clock, 0.1)
    assert node.mode == "idle"
    assert node.m.last("setVelocityControlFlag") == ("setVelocityControlFlag", False)
    node.m.calls.clear()
    node.cmd_cb(_twist(0.3))
    node.stream()
    assert "setVelocity" not in node.m.names()


def test_never_navigator_only_warns(node, clock):
    node.m.cm_fixed = CM_REMOTE                 # 플래그를 켜도 Navigator 로 안 바뀌는 펌웨어라면
    make_ready(node, clock)
    run(node, clock, 5.0)
    assert node.mode == "ready" and node.nav_warned   # 멈추지 않고 경고만


def test_yield_to_remote_disabled(node, clock):
    node.yield_to_remote = False
    make_ready(node, clock)
    run(node, clock, 0.3)
    node.m._control_mode = CM_REMOTE
    run(node, clock, 1.0)
    assert node.mode == "ready"


# ---------------------------------------------------------------- 속도 명령
def test_cmd_timeout_sends_zero(node, clock):
    make_ready(node, clock)
    node.cmd_cb(_twist(0.3, 0.1, 0.5))
    node.stream()
    assert node.m.last("setVelocity") == ("setVelocity", 0.3, 0.1, 0.5)
    clock.t += 0.4
    node.stream()
    assert node.m.last("setVelocity") == ("setVelocity", 0.3, 0.1, 0.5)
    clock.t += 0.2                              # 0.6 s > cmd_timeout 0.5
    node.stream()
    assert node.m.last("setVelocity") == ("setVelocity", 0.0, 0.0, 0.0)


def test_velocity_clamped_and_nan_ignored(node, clock):
    make_ready(node, clock)
    node.cmd_cb(_twist(9.0, -9.0, 9.0))
    node.stream()
    assert node.m.last("setVelocity") == ("setVelocity", 0.4, -0.25, 1.0)
    node.cmd_cb(_twist(float("nan"), 0.0, 0.0))
    node.stream()
    assert node.m.last("setVelocity") == ("setVelocity", 0.4, -0.25, 1.0)  # 이전 값 유지
    node.cmd_cb(_twist(math.inf, 0.0, 0.0))
    node.stream()
    assert all(math.isfinite(v) for v in node.m.last("setVelocity")[1:])


def test_no_streaming_outside_ready(node, clock):
    run(node, clock, 0.2)
    node.cmd_cb(_twist(0.3))
    node.stream()
    assert "setVelocity" not in node.m.names()


# ---------------------------------------------------------------- 앉기 / 힘풀기 / 종료
def test_sit_stops_then_lies_down(node, clock):
    make_ready(node, clock)
    node.request("sit")
    assert node.m.calls[:2] == [("setVelocity", 0.0, 0.0, 0.0), ("setVelocityControlFlag", False)]
    run(node, clock, 2.5)
    assert node.mode == "idle" and node.m.getCurrentMotionState() == MS_LYING


def test_damping(node, clock):
    make_ready(node, clock)
    ok, _ = node.request("damping")
    assert ok and node.mode == "idle"
    assert node.m.names()[:3] == ["setVelocity", "setVelocityControlFlag", "setDamping"]


def test_shutdown_sits_robot(node, clock):
    make_ready(node, clock)
    node.shutdown_robot()
    assert node.m.getCurrentMotionState() == MS_LYING
    assert node.m.names()[:2] == ["setVelocity", "setVelocityControlFlag"]


def test_shutdown_read_only_sends_nothing(node, clock):
    node.allow_motion = False
    run(node, clock, 0.5)
    node.m.calls.clear()
    node.shutdown_robot()
    assert node.m.calls == []


def _twist(x=0.0, y=0.0, z=0.0):
    from geometry_msgs.msg import Twist
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, z
    return t


# ---------------------------------------------------------------- 실행 중 속도 상한 변경
def test_runtime_speed_limits(node, clock):
    from rclpy.parameter import Parameter
    r = node.set_parameters([Parameter("max_vx", value=0.6)])
    assert r[0].successful and node.max_vx == 0.6
    make_ready(node, clock)
    node.cmd_cb(_twist(9.0))
    node.stream()
    assert node.m.last("setVelocity")[1] == 0.6
    bad = node.set_parameters([Parameter("max_vx", value=3.0)])      # SDK 상한 1.5 초과
    assert not bad[0].successful and node.max_vx == 0.6
    bad = node.set_parameters([Parameter("cmd_timeout", value=5.0)])
    assert not bad[0].successful and node.cmd_timeout == 0.5
