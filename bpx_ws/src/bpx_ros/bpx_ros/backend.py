"""BPX 제어 백엔드: 실제 SDK(bpx_sdk.MotionLevelControl) 또는 로봇 없이 쓰는 Fake.

두 백엔드는 같은 메서드 이름(SDK API 그대로)을 가진다 → 브리지 코드는 구분하지 않는다.
SDK 사실관계 (bpx_sdk_open 1.0.9 헤더/README, robot-mirrorme-bpx 실기 검증 기준):
  - 쿼터니언 순서 x, y, z, w  (getImuQuat, LegOdom.orientation)
  - LegOdom: +x 전진, +y 왼쪽, +yaw 좌회전 (2026-09-08 실기 확인)
  - MotionState: LyingDown=0 StandingUp=1 Passive=2 SitDown=3 Motion=6
  - 관절 인덱스 0~11 = FL(roll,pitch,knee) FR HL HR  (URDF 관절명과 동일 순서)
"""
import math
import time

MS_LYING = 0
MS_STANDING_UP = 1
MS_PASSIVE = 2
MS_SITDOWN = 3
MS_MOTION = 6

# ControlMode: 현재 속도 제어 주체 (SDK 1.0.9 README / request_robot_state.h)
CM_UNKNOWN = 0
CM_REMOTE = 1       # 앱 / 리모컨
CM_NAVIGATOR = 2    # SDK (내비게이션)

JOINT_NAMES = [
    f"{leg}_{j}_joint"
    for leg in ("fl", "fr", "hl", "hr")
    for j in ("hip_roll", "hip_pitch", "knee")
]


def make_backend(kind, robot_ip, state_port, state_rate, cmd_rate):
    if kind == "fake":
        m = FakeMotion()
    elif kind == "sdk":
        import bpx_sdk  # third_party/bpx_sdk 의 wheel (cp310 = ROS 2 Humble 파이썬)
        m = bpx_sdk.MotionLevelControl()
    else:
        raise ValueError(f"unknown backend: {kind} (sdk|fake)")
    m.setRobotIp(robot_ip)
    m.setRobotStateUploadPort(state_port)
    m.setRobotStateUploadRate(state_rate)
    m.setMotionCommandRate(cmd_rate)
    # connect 가 주기 명령 송신을 시작하기 전에 '속도 제어 아님'을 명시 (제조사 Robonix posture 세션과 같음)
    m.setVelocityControlFlag(False)
    return m


class FakeMotion:
    """로봇 없이 브리지/launch/RViz/Nav2 연동을 시험하기 위한 가짜 MotionLevelControl.

    상태 전이(서기 2초, 앉기 2초)와 속도 적분 odom 만 흉내낸다. 물리/보행은 없다.
    ControlMode 는 속도 플래그를 켜면 Navigator, 끄면 RemoteControl 로 가정한다 (실기 미확인).
    """

    STAND_POSE = (0.0, 0.8, -1.5)
    LIE_POSE = (0.0, 1.2, -2.7)
    TRANSITION_S = 2.0

    def __init__(self):
        self._connected = False
        self._state = MS_LYING
        self._target = None          # (목표 상태, 완료 시각)
        self._current = -1.5         # 배터리 전류 (양수 = 충전 중)
        self._gait = 0               # 0 Walk, 8 Running
        self._vel_flag = False
        self._control_mode = CM_REMOTE
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_t = 0.0
        self._x = self._y = self._yaw = 0.0
        self._t = time.monotonic()
        self.__version__ = "fake"

    # --- 설정 / 연결 ---
    def setRobotIp(self, ip): pass
    def setRobotStateUploadPort(self, port): pass
    def setRobotStateUploadRate(self, hz): pass
    def setMotionCommandRate(self, hz): pass
    def connect(self):
        self._connected = True
        return True
    def disconnect(self): self._connected = False
    def isConnected(self): return self._connected

    # --- 명령 ---
    def setVelocityControlFlag(self, enabled):
        self._vel_flag = bool(enabled)
        self._control_mode = CM_NAVIGATOR if enabled else CM_REMOTE
    def setVelocity(self, x, y, yaw):
        self._step()
        self._cmd = (float(x), float(y), float(yaw))
        self._cmd_t = time.monotonic()
        return True
    def setStandUp(self):
        self._step()
        if self._state != MS_MOTION and self._target is None:
            self._state = MS_STANDING_UP
            self._target = (MS_MOTION, time.monotonic() + self.TRANSITION_S)
        return True
    def setSitDown(self):
        self._step()
        if self._state != MS_LYING and (self._target is None or self._target[0] != MS_LYING):
            self._state = MS_SITDOWN
            self._target = (MS_LYING, time.monotonic() + self.TRANSITION_S)
        return True
    def setDamping(self):
        self._state, self._target = MS_PASSIVE, None
        return True
    def setZeroPositionsFlag(self): pass

    # --- 상태 ---
    def _step(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        if self._target and now >= self._target[1]:
            self._state, self._target = self._target[0], None
        if self._state == MS_MOTION and self._vel_flag and now - self._cmd_t < 0.5:
            vx, vy, wz = self._cmd
            c, s = math.cos(self._yaw), math.sin(self._yaw)
            self._x += (vx * c - vy * s) * dt
            self._y += (vx * s + vy * c) * dt
            self._yaw += wz * dt
            return vx, vy, wz
        return 0.0, 0.0, 0.0

    def _quat(self):
        return [0.0, 0.0, math.sin(self._yaw / 2), math.cos(self._yaw / 2)]

    def getCurrentMotionState(self):
        self._step()
        return self._state
    def getCurrentGait(self): return self._gait
    def getSubGait(self): return 0
    def getLastGait(self): return 0
    def getLastMotionState(self): return MS_LYING
    def setWalk(self): self._gait = 0; return True
    def setRunning(self): self._gait = 8; return True
    def getMaxVelocity(self):
        return [1.5, 0.5, 2.0] if self._current < 0 else [0.0, 0.0, 0.0]   # 실기: 충전 중엔 0 으로 잠금
    def getDriverTemperature(self): return [round(38.0 + 0.3 * i, 1) for i in range(12)]
    def getImuRpy(self): return [0.0, 0.0, self._yaw]
    def getRobotVersion(self): return (1, 0, 14, 0, 20260818, 0)
    def getControlMode(self): return self._control_mode
    def getMotorTemperature(self): return [round(35.0 + 0.5 * i, 1) for i in range(12)]
    def getLegOdom(self):
        vx, vy, wz = self._step()
        return {
            "position": [self._x, self._y, 0.0],
            "orientation": self._quat(),
            "velocity_body": [vx, vy, 0.0],
            "angular_velocity": [0.0, 0.0, wz],
        }
    def getImuQuat(self): return self._quat()
    def getImuOmega(self): return [0.0, 0.0, self._step()[2]]
    def getImuAcc(self): return [0.0, 0.0, 9.81]
    def getJointPosition(self):
        self._step()
        pose = self.STAND_POSE if self._state in (MS_MOTION, MS_STANDING_UP) else self.LIE_POSE
        return [v for _ in range(4) for v in pose]
    def getJointVelocity(self): return [0.0] * 12
    def getJointTorque(self): return [0.0] * 12
    def getBatteryLevel(self): return 87
    def getBatteryCurrent(self): return self._current   # 실기: 방전 음수(-0.64), 충전 양수(+2.38)
    def getRobotModel(self): return 1
    def getRobotSerialNumber(self): return "FAKE00000"
