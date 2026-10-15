# BPX ROS 2 프로젝트

MirrorMe(mirrormetech.com) **BPX 4족로봇**을 로봇에 올린 **미니PC**(Ubuntu 22.04 + ROS 2 Humble)에서 전부 제어한다.
구조는 이전 프로젝트 Zishen_ROS(zsl-1w 4족 + Livox Mid-360, FAST-LIO / NDT / Nav2 MPPI / cmd_vel_bridge / map_to_odom / 관제 M20 브리지)를 따른다.
미니PC는 새 PC → genisom 벤더 스택(navigo_* Nav2 포크, robots_dog_msgs) 없음. Nav2·SLAM 은 표준/오픈소스로 구성.

## 로봇 사실관계 (검증된 출처 기준)
- 12 DOF (다리당 hip_roll, hip_pitch, knee). BPX 15.2 kg / BPX-Pro 17.6 kg, 최대 탑재 5 kg
- 관절 순서(SDK 인덱스 0~11) = FL, FR, HL, HR 각 (roll, pitch, knee) = URDF 관절명 `{fl,fr,hl,hr}_{hip_roll,hip_pitch,knee}_joint`
- SDK `bpx_sdk_open` 1.0.9: 자체 TCP/UDP (ROS/DDS 아님), 코어 바이너리 전용, **인증 없음**
  - 로봇 IP: RJ45 직결 `10.21.20.1`, 로봇 핫스팟 `10.21.40.1` (SSID BPX_XXXXXX / 12345678)
  - 로봇→PC 상태 UDP **9873**, 고속 관절 UDP 7895 → 방화벽 허용 필요
  - 쿼터니언 순서 **x,y,z,w**. LegOdom: +x 전진, +y 왼쪽, +yaw 좌회전 (제조사 실기 확인)
  - MotionState: LyingDown=0, StandingUp=1, Passive=2, SitDown=3, Motion=6
  - 주행 순서: connect → setStandUp 반복 → state==6 → setVelocityControlFlag(True) → setVelocity
  - 재연결 후 SDK 는 0 아닌 명령을 재전송하지 않음 → 서기 절차부터 다시
  - setZeroPositionsFlag 는 네 다리 발·종아리·무릎 접지(엎드림)에서만
- 미확인: 관절 부호·영점, IMU 장착 frame, 배터리 전류/토크 단위, 미니PC IP 할당 방식, 외부 전원 출력(없음 → 별도 배터리)
- 참고 저장소: github.com/mirrormerobotics (bpx_sdk_open, BPX, BPX-Pro, mirrorme_rl_train, robot-mirrorme-bpx, primitive-mirrorme-bpx-quadruped-rbnx, bpx-mindplus-extension)

## 폴더
- `bpx_ws/src/bpx_ros/` ROS 2 패키지(ament_python)
  - `bpx_ros/bridge.py` 핵심 노드: /cmd_vel → setVelocity, LegOdom → `/odom/mc_odom` + TF odom→base_link, /joint_states, /imu/data, /battery_state, /bpx/status, 서비스 `~/stand ~/sit ~/damping`, 토픽 `/bpx/motion`(관제 훅)
  - `bpx_ros/backend.py` 실제 SDK / Fake 백엔드 (같은 메서드명)
  - `bpx_ros/map_to_odom.py` Zishen 이식
  - `launch/bringup.launch.py` (backend:=sdk|fake, allow_motion, robot_ip, cmd_topic, rviz)
  - `config/bridge.yaml`, `urdf/bpx.urdf`(base_link 루트 추가), `meshes/`
- `scripts/bpx_cmd.py` ROS 없이 단발 동작 (check/watch/stand/sit/damping/zero, --fake)
- `setup/` setup_minipc.sh, bpx_env.sh(ROS_DOMAIN_ID=90), net_bpx.sh(nmcli 고정 IP: eno1=로봇, enp4s0=라이다), netplan-bpx.yaml(참고용)
- `third_party/bpx_sdk/` SDK wheel (cp310 x86_64 / aarch64)
- `docker/` 로봇 없이 시험 (test_fake.sh, 18항목)
- `docs/` 매뉴얼 PDF, 업무일지

## 작업 규칙
- **안전 기본값 유지**: allow_motion 기본 false, auto_stand 기본 false, /cmd_vel 타임아웃 0.5 s, 속도 제한(0.4/0.25/1.0)
- 실기에서 로봇을 움직이는 명령(stand, cmd_vel 발행, zero, damping)은 **사용자 확인 후** 실행. 읽기(check, topic echo)는 자유
- SDK 소유는 한 프로세스만 (브리지 실행 중 bpx_cmd.py 금지, 앱/Mind+ 동시 조종 금지)
- 변경 후 `colcon build --symlink-install` 및 fake 백엔드로 먼저 확인
- 업무일지는 `docs/업무일지_YYYY-MM-DD.md`, "Noah 일일 업무 보고" 형식, "~했다" 체

## 진행 상황
- 2026-10-13: 자료 분석, bpx_ros 패키지·설치 스크립트·Docker 테스트(18/18) 완료. 실기 미실행
- 2026-10-15: 새 미니PC에 Linux 설치 + SSH 설정. setup_minipc.sh 실행(ROS 2 Humble, bpx_sdk 1.0.9, 빌드) → fake 시험 18/18. 유선 NIC = eno1, enp4s0 (netplan 미적용)
- 다음 단계: 라이다 선정(3D Mid-360 vs 2D) → 드라이버/매핑/위치추정/표준 Nav2
