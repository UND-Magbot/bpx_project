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
- 전원 (사용자 매뉴얼 p13 그림 + 판매처 3digital.tech, 제조사 사양서 아님)
  - 배터리 8S2P NMC 18650, 공칭 28.8 V (24.0~33.6 V), 5.8 Ah / 167 Wh, 최대 방전 20 A, 무부하 약 1.5 h
  - 후면 **BAT Power Port = GND / 33V** (배터리 직결 추정 → 24~33.6 V 변동, 허용 전류 미표기). 5V: 전면 USB·LED, S-Bus
  - 미니PC = MSI PRO Cubi Z AI 8M (Ryzen 7 255), 어댑터 120 W, USB-C PD 입력 최대 100 W → BAT 포트에서 받으려면 광입력 DC-DC 필요
- 리모컨: 펌웨어가 S-Bus(RC 수신기)·USB 게임패드(옵션)·앱을 직접 처리. SDK 에 스틱 입력 API 없음, `getControlMode()` 로 RemoteControl=1 / Navigator=2 확인만 가능. 앱/리모컨과 SDK 우선순위 문서 없음
- **실기 확인 (2026-10-19 첫 RJ45 연결, 엎드림 상태·읽기 전용)**: eno1 ↔ 로봇 10.21.20.1 ping 0.2 ms, 미니PC 고정 IP 10.21.20.10 동작
  - 상태 50 Hz(odom/관절/IMU), 배터리 1 Hz. 모터 온도 28.0 → ℃
  - BPX IMU 가속도 단위 **m/s²** (|a|=9.78, Livox 의 g 와 다름). 엎드림 roll/pitch ≈ 0 → IMU z 위쪽, base_link 와 수평. IMU yaw = LegOdom yaw
  - 관절(엎드림) pitch +1.08~1.21, knee -2.70~-2.83 rad → URDF/Fake 엎드림 자세와 부호 일치
  - 배터리 전류 대기 +2.38 (A 라면 약 69 W, 충전 시와 비교해 부호·단위 확정 필요)
  - **ControlMode = None** (이 펌웨어는 미제공) → 브리지 탈취 감지는 경고만, 동작 안 함
- 미확인: IMU x/y 축 방향(전진 시 확인), 서 있을 때 관절 영점, 토크 단위, 배터리 전류 부호, BAT 포트 허용 전류·커넥터 규격
- 참고 저장소: github.com/mirrormerobotics (bpx_sdk_open, BPX, BPX-Pro, mirrorme_rl_train, robot-mirrorme-bpx, primitive-mirrorme-bpx-quadruped-rbnx, bpx-mindplus-extension)

## 폴더
- `bpx_ws/src/bpx_ros/` ROS 2 패키지(ament_python)
  - `bpx_ros/bridge.py` 핵심 노드: /cmd_vel → setVelocity, LegOdom → `/odom/mc_odom` + TF odom→base_link, /joint_states, /imu/data, /battery_state, /bpx/status, 서비스 `~/stand ~/sit ~/damping`, 토픽 `/bpx/motion`(관제 훅)
  - `bpx_ros/backend.py` 실제 SDK / Fake 백엔드 (같은 메서드명)
  - `bpx_ros/map_to_odom.py` Zishen 이식
  - `launch/bringup.launch.py` (backend:=sdk|fake, allow_motion, robot_ip, cmd_topic, rviz, foxglove)
    - foxglove_bridge 는 **보기 전용**(capabilities=connectionGraph,assets) 유지. clientPublish/services 켜면 WiFi 에서 누구나 조종 가능해짐
  - `launch/nav.launch.py` 표준 Nav2 (MPPI Omni, NavFn, 빈 지도 `maps/empty.yaml`, map→odom 고정). `config/nav2.yaml`
    - 라이다 전이라 장애물 레이어 없음 → **실기 자율주행 금지**, fake 시험 전용. velocity_smoother 출력 = /cmd_vel
  - `config/bridge.yaml`, `urdf/bpx.urdf`(base_link 루트 추가), `meshes/`
- `bpx_ws/src/bpx_web/` **로봇 관리자 웹** (FastAPI + rclpy 한 프로세스, http://<미니PC>:8080, 인증 없음 = 사내 시험망 전용)
  - 상태 대시보드, 조종 모드 스위치(브리지 `~/set_motion`), 일어서기/앉기/힘 풀기, 관절 영점(`~/zero`, 엎드림에서만, 확인 창), 비상 정지(damping), 가상 조이스틱·WASDQE, 지도·궤적·Nav2 목표
  - 조이스틱은 누르는 동안만 /cmd_vel 송신, 놓으면 0 한 번 (Nav2 명령을 덮지 않음). 0.3 s 끊기거나 화면이 닫히면 서버가 0. 속도 상한 서버에서도 적용
  - 조종 모드를 끄면 브리지가 속도 0 + 플래그 해제 (서 있던 로봇은 펌웨어가 세워 둠 → 끄기 전에 앉힐 것)
  - uvicorn 은 `ws='wsproto'` 필수 (Ubuntu 22.04 websockets 9.1 이 Python 3.10 과 호환 안 됨)
  - 자동 실행 서비스가 `web:=true` 로 띄움. fake 시험: lidar_ws/webtest/scenario.py (10항목)
- `scripts/bpx_cmd.py` ROS 없이 단발 동작 (check/watch/stand/sit/damping/zero, --fake)
- `setup/` setup_minipc.sh, bpx_env.sh(ROS_DOMAIN_ID=90), net_bpx.sh(nmcli 고정 IP: eno1=로봇, enp4s0=라이다), netplan-bpx.yaml(참고용)
- `third_party/bpx_sdk/` SDK wheel (cp310 x86_64 / aarch64)
- `docker/` 로봇 없이 시험 (test_fake.sh, 18항목)
- `docs/` 매뉴얼 PDF, 업무일지

## 작업 규칙
- **안전 기본값 유지**: allow_motion 기본 false, auto_stand 기본 false, /cmd_vel 타임아웃 0.5 s, 속도 제한(0.4/0.25/1.0)
- 실기에서 로봇을 움직이는 명령(stand, cmd_vel 발행, zero, damping)은 **사용자 확인 후** 실행. 읽기(check, topic echo)는 자유
- SDK 소유는 한 프로세스만 (브리지 실행 중 bpx_cmd.py 금지, 앱/Mind+ 동시 조종 금지)
  - 미니PC는 부팅 시 `bpx-bridge.service`(읽기 전용)가 자동 실행됨 → bpx_cmd.py·수동 launch 전에 `sudo systemctl stop bpx-bridge`
- 변경 후 `colcon build --symlink-install` + `colcon test --packages-select bpx_ros`(상태머신·조종권 17항목) + fake 백엔드로 먼저 확인
  - 수동 fake 시험은 `ROS_DOMAIN_ID`를 90 아닌 값으로 (서비스 브리지와 토픽 섞임 방지)
  - 비대화형 셸에서 `ros2 launch ... &` 로 띄우면 launch 가 SIGINT 무시 상태를 상속 → `kill -INT <launch>` 가 안 먹고 노드가 남음. 시험 정리는 노드 PID 에 직접 SIGINT
- 업무일지는 `docs/업무일지_YYYY-MM-DD.md`, "Noah 일일 업무 보고" 형식, "~했다" 체

## 진행 상황
- 2026-10-13: 자료 분석, bpx_ros 패키지·설치 스크립트·Docker 테스트(18/18) 완료. 실기 미실행
- 2026-10-15: 새 미니PC에 Linux 설치 + SSH 설정. setup_minipc.sh 실행(ROS 2 Humble, bpx_sdk 1.0.9, 빌드) → fake 시험 18/18. nmcli 고정 IP 저장(eno1=로봇, enp4s0=라이다), git 저장소, RViz fake 확인, systemd 자동 실행(읽기 전용), 상태머신 단위시험 14/14, Foxglove 원격 보기(8765, 보기 전용), 표준 Nav2 fake 주행 2/2 성공
- 2026-10-16: ControlMode 감시, Mid-360S 드라이버·FAST-LIO·NDT 위치추정 설치, 공개 Mid-360 bag 으로 매핑·지도 수평 보정·위치추정 검증 (하드웨어 없이)
- 라이다 결정: **Livox Mid-360S** (9~27 V, 권장 12 V, 평상 6.5 W, 기동 18 W 약 8초, 영하 자가가열 14 W, 100BASE-TX, M12 12핀, IMU ICM40609)
  - BPX BAT 포트(최대 33.6 V)는 27 V 초과 → **직결 금지**, 12 V DC-DC 경유
  - 드라이버: `setup/install_livox.sh` (Livox-SDK2 v1.3.1 → /usr/local, livox_ros_driver2 1.2.6 → `lidar_ws/`, git 제외)
  - `ros2 launch bpx_ros lidar.launch.py lidar_ip:=192.168.1.1XX` (XX = SN 끝 두 자리, host 192.168.1.50 = enp4s0)
    → /livox/lidar (CustomMsg, xfer_format:=0 이면 PointCloud2), /livox/imu, TF base_link→livox_frame(장착 위치 미측정, 0)
  - 유선 NIC 는 ignore-carrier 로 랜선 없어도 IP 유지 (드라이버가 시작 때 bind 실패하면 재시도 안 함)
  - 라이다 미연결 상태의 드라이버는 SIGINT/SIGTERM 을 무시 → 노드에 직접 신호 보낼 땐 SIGKILL 필요
  - **Livox IMU 가속도 단위 = g** (정지 시 |a|≈1.0). FAST-LIO 는 내부 정규화로 무관, 다른 패키지에 줄 땐 ×9.81
  - FAST-LIO `camera_init` 은 출발 시 센서 자세 기준(중력 정렬 아님) → 센서가 기울면 지도도 기움.
    2D 지도/NDT/Nav2 로 넘길 때 중력 정렬 필요 (AIST bag: 초기 7.3° 기울기로 고도 46 m 왜곡, 정렬 후 ~10 m)
  - FAST-LIO 매핑: `mapping.launch.py map_name:=X` → `/map_save` → `~/bpx/maps/X.pcd`
  - PointCloud2(xfer_format 0) bag → CustomMsg 변환: `scripts/livox_pc2_to_custom.py <in> <out>`
  - 공개 bag 검증: AIST "Driving SLAM Test with Livox MID360" (Zenodo 14841855, CC BY 4.0) 277 s / 1.07 km 루프 매핑 정상
    bag 은 `lidar_ws/bags/` 에 둔다 (/tmp 는 재부팅 때 지워짐)
  - 지도 수평 보정: mapping.launch.py 가 시작 1초 IMU 로 `<map>.gravity.json` 저장 → `scripts/align_map_gravity.py <map>.pcd` → `<map>_level.pcd`
- 위치추정: lidar_localization_ros2 (main ba9f7f06db, NDT_OMP) + ndt_omp_ros2, `lidar_ws` (install_livox.sh)
  - Nav2 모드(기본) = map→odom 발행, odom→base_link 는 브리지 LegOdom. bag 시험처럼 odom 없으면 `enable_map_odom_tf:=false`
  - 입력은 PointCloud2 → 위치추정 때 lidar.launch.py `xfer_format:=0` (매핑 FAST-LIO 는 1)
  - 4족+Mid-360 참고: 패키지 docs/go2_mid360_quickstart.md
  - 위치 출력 토픽은 `/localization/pose_with_covariance` (이 launch 에선 /pcl_pose 발행 안 함). 갱신은 스캔 주기(10 Hz), 발행은 20 Hz
  - AIST bag 검증(2026-10-16): 수평 보정 지도 + NDT_OMP(IMU 기능 끔), 초기 자세 = 중력 보정 회전.
    277 s 전 구간 추적, 거부 0건, 공백 없음, 정합 ~8 ms, 고도 -3.2~6.8 m(보정 FAST-LIO 와 일치), 끝점 FAST-LIO 와 0.3 m 이내.
    한계: 같은 주행으로 만든 지도라 다른 날·다른 경로에서의 성능은 실기에서 확인 필요. IMU 사전적분은 Livox g 단위 설정 후 켤 것
- **빌드 메모리 주의**: NDT/PCL 컴파일은 파일 하나가 최대 3.6 GB. `-j8` 로 빌드하다 미니PC(14 GB)가 두 번 멈춰 재부팅됨 (2026-10-16).
  lidar_ws 빌드는 `MAKEFLAGS=-j2 --parallel-workers 1` + `systemd-run -p MemoryMax=9G` 로 (install_livox.sh 에 반영)
- 다음 단계: Mid-360S 드라이버(Livox-SDK2 + livox_ros_driver2) → FAST-LIO 매핑 / NDT 위치추정 → Nav2 장애물 레이어
- 2026-10-19: 첫 실기 연결(읽기 전용 실측), 관리자 웹. 웹 일어서기 시도 → 15 s 동안 motion_state 가 6 이 안 돼 타임아웃.
  제조사 예제/Mind+ 는 서기 전에 관절 영점(setZeroPositionsFlag)을 먼저 함 → 웹에 관절 영점 버튼 추가, 브리지가 motion_state 변화를 로그로 남김
  영점 후에도 motion_state 가 0 에서 전혀 안 바뀜 (StandingUp 1 도 안 거침) → 로봇이 동작 명령 자체를 받지 않음
  - 로봇 펌웨어 **1.0.14 (2026-08-18)**, SN/model/ControlMode = None (SDK 1.0.9 신기능 미지원), 충전기 [0,0]
  - 제조사 실기 서기 검증(robot-mirrorme-bpx POSTURE_VALIDATION)은 펌웨어 **1.0.15** + SDK 1.0.8, 영점 호출 없이 성공
  - 후보: ① 펌웨어 업데이트(앱 OTA) ② 앱/리모컨이 조종 중 (검증 절차 전제: 리모컨·앱·다른 SDK 모두 정지)
  - **패킷 캡처로 확인 (18:26)**: 통신 정상. 로봇→PC UDP 9873 상태, PC↔로봇 TCP 10860 세션, PC→로봇 **UDP 9527 명령 50 Hz**.
    일어서기 시 명령 첫 필드 06→01, 요청 카운터가 0.2~0.3 s 마다 증가(18 s 동안 69회) → **요청은 로봇에 도착, 로봇이 무시** (motion_state 0 유지)
    다음 판단: 앱으로는 일어서는지 / Fault Diagnosis / 펌웨어 업데이트. 증거 lidar_ws/bpx_stand.pcap
