#!/usr/bin/env python3
"""BPX 단발 동작 스크립트 (ROS 없이 SDK 만 사용) — Zishen robot_*.py 와 같은 용도.

  python3 bpx_cmd.py check            # 연결 + 상태 출력 (로봇 안 움직임)
  python3 bpx_cmd.py watch            # 상태 1초마다 출력 (Ctrl+C 종료)
  python3 bpx_cmd.py stand            # 일어서기 (Motion 상태 될 때까지)
  python3 bpx_cmd.py sit              # 앉기 (LyingDown 될 때까지)
  python3 bpx_cmd.py damping          # 소프트 힘풀기 (하드웨어 E-stop 아님)
  python3 bpx_cmd.py zero --yes       # 관절 영점 (네 다리 발/종아리/무릎이 바닥에 닿은 엎드린 자세에서만!)

옵션: --ip 10.21.20.1 (핫스팟이면 10.21.40.1)   --fake (로봇 없이 시험)
bpx_bridge 가 실행 중이면 쓰지 말 것 (SDK 소유는 한 곳만).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bpx_ws", "src", "bpx_ros"))
from bpx_ros.backend import MS_LYING, MS_MOTION, make_backend  # noqa: E402

STATE_NAMES = {0: "LyingDown", 1: "StandingUp", 2: "Passive", 3: "SitDown", 6: "Motion"}


def wait_connect(m, timeout=10.0):
    if not m.connect():
        sys.exit("connect() 실패 — ping, 랜선, 방화벽(UDP 9873) 확인")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if m.isConnected() and m.getLegOdom() is not None:
            return
        time.sleep(0.1)
    sys.exit("연결은 됐지만 상태(UDP)가 안 들어옴 — 미니PC 방화벽에서 UDP 9873 허용 필요")


def show(m):
    ms = m.getCurrentMotionState()
    rpy = getattr(m, "getImuRpy", lambda: None)()
    print(f"model={m.getRobotModel()} sn={m.getRobotSerialNumber()} "
          f"state={ms}({STATE_NAMES.get(ms, '?')}) battery={m.getBatteryLevel()}%")
    print(f"  imu_rpy={rpy}")
    print(f"  odom={m.getLegOdom()}")
    print(f"  joints={m.getJointPosition()}")


def until(m, method, target, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        getattr(m, method)()
        if m.getCurrentMotionState() == target:
            return True
        time.sleep(0.2)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["check", "watch", "stand", "sit", "damping", "zero"])
    ap.add_argument("--ip", default="10.21.20.1")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--yes", action="store_true", help="zero 명령 확인")
    a = ap.parse_args()

    if a.cmd == "zero" and not a.yes:
        sys.exit("zero 는 엎드린 자세(발·종아리·무릎 접지) 확인 후 --yes 를 붙여 실행")

    m = make_backend("fake" if a.fake else "sdk", a.ip, 9873, 50, 50)
    wait_connect(m)
    try:
        if a.cmd == "check":
            show(m)
        elif a.cmd == "watch":
            while True:
                show(m)
                time.sleep(1.0)
        elif a.cmd == "stand":
            ok = until(m, "setStandUp", MS_MOTION)
            print("서기 완료" if ok else "서기 타임아웃 (앱 Fault Diagnosis 확인)")
        elif a.cmd == "sit":
            m.setVelocity(0.0, 0.0, 0.0)
            m.setVelocityControlFlag(False)
            ok = until(m, "setSitDown", MS_LYING)
            print("앉기 완료" if ok else "앉기 타임아웃")
        elif a.cmd == "damping":
            m.setVelocityControlFlag(False)
            m.setDamping()
            print("damping 요청")
        elif a.cmd == "zero":
            m.setZeroPositionsFlag()
            time.sleep(1.0)
            print("관절 영점 요청 완료")
    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()


if __name__ == "__main__":
    main()
