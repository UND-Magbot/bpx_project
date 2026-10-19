# BPX ROS 2

MirrorMe **BPX 4족로봇**을 **미니PC(IPC)** 에서 ROS 2 Humble 로 구동하기 위한 패키지·스크립트.
구조는 `../Zishen_ROS` (zsl-1w + Mid-360) 를 그대로 따르고, 로봇 SDK 부분만 `bpx_sdk` 로 교체했다.

## 시스템 구성

```
[Windows PC] --(WiFi)--> [미니PC: Ubuntu 22.04 + ROS 2 Humble]  ← 모든 노드 실행
                              ├── 유선 NIC → BPX RJ45  (로봇 10.21.20.1, 미니PC 10.21.20.10)
                              └── 유선 NIC → LiDAR     (예정)
```

- **로봇 SDK**: `bpx_sdk_open` 1.0.9 (Python 3.10 wheel, `third_party/bpx_sdk/`). 자체 TCP 명령 + **로봇→미니PC UDP 9873** 상태
- **TF**: `map →(map_to_odom)→ odom →(bpx_bridge)→ base_link → torso → 다리…` (Zishen 과 동일)
- **odom 토픽**: `/odom/mc_odom` (Zishen 스택과 같은 이름 → map_to_odom / Nav2 params 재사용)

## 폴더

| 경로 | 내용 |
|---|---|
| `bpx_ws/src/bpx_ros/bpx_ros/bridge.py` | **핵심.** `/cmd_vel` → `setVelocity`, LegOdom → `/odom/mc_odom` + TF, `/joint_states`, `/imu/data`, `/battery_state`, `/bpx/status`, 서비스 `stand/sit/damping` |
| `bpx_ws/src/bpx_ros/bpx_ros/backend.py` | 실제 SDK / **Fake** 백엔드 (로봇 없이 시험) |
| `bpx_ws/src/bpx_ros/bpx_ros/map_to_odom.py` | Zishen `map_to_odom.py` 이식 (토픽 파라미터화) |
| `bpx_ws/src/bpx_ros/launch/bringup.launch.py` | bridge + robot_state_publisher (+ rviz) |
| `bpx_ws/src/bpx_ros/config/bridge.yaml` | IP·포트·속도제한·토픽 |
| `bpx_ws/src/bpx_ros/urdf/bpx.urdf`, `meshes/` | mirrormerobotics/BPX URDF + `base_link` 루트 추가 |
| `scripts/bpx_cmd.py` | ROS 없이 단발 동작: `check / watch / stand / sit / damping / zero` |
| `setup/setup_minipc.sh` | 미니PC 설치 (ROS 2, SDK, 방화벽, 빌드, bashrc) |
| `setup/netplan-bpx.yaml` | 유선 NIC 고정 IP 예시 |
| `setup/bpx_env.sh` | 환경변수 (`ROS_DOMAIN_ID=90`) |
| `docker/` | 이 Windows PC 에서 로봇 없이 시험 (`ros:humble`) |
| `docs/` | BPX 사용자/조립 매뉴얼 PDF |

## 미니PC 설치

```bash
# bpx 폴더를 미니PC ~/bpx 로 복사 후
bash ~/bpx/setup/setup_minipc.sh
sudo cp ~/bpx/setup/netplan-bpx.yaml /etc/netplan/60-bpx.yaml   # NIC 이름 수정 후
sudo netplan apply
ping 10.21.20.1
```

## 실기 첫 연결 순서 (안전 단계)

1. **앱에서 먼저**: Fault Diagnosis → Reset Joints (매뉴얼 p5). 이후 앱은 끄기 — SDK 와 동시 조종 금지
2. `python3 scripts/bpx_cmd.py check` — 연결·배터리·odom 확인 (안 움직임)
3. `ros2 launch bpx_ros bringup.launch.py` — **읽기 전용**(allow_motion=false). RViz 에서 로봇 자세가 실물과 같은지, 손으로 밀었을 때 odom 방향(+x 전진, +y 왼쪽) 확인
4. `ros2 launch bpx_ros bringup.launch.py allow_motion:=true`
   `ros2 service call /bpx_bridge/stand std_srvs/srv/Trigger`
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard` (저속부터)
5. 비상: `ros2 service call /bpx_bridge/damping std_srvs/srv/Trigger` 또는 브리지 Ctrl+C(정지 후 앉기). **하드웨어 E-stop 은 후면 E-Stop 포트**

## 로봇 관리자 웹

브라우저로 `http://192.168.30.52:8080` (미니PC WiFi 주소). 부팅 시 자동 실행된다.

- 상태: 연결, 동작 상태, 배터리, 모터 온도, 속도, 위치, 기록
- **조종 모드** 스위치를 켜야 버튼·조이스틱이 로봇에 전달된다 (부팅 직후는 항상 꺼짐 = 읽기 전용)
- 일어서기 / 앉기 / 힘 풀기 / 비상 정지(소프트웨어 damping, 하드웨어 E-Stop 은 로봇 후면)
- 원격 조종: 화면 조이스틱 또는 키보드 W/S(전후) A/D(회전) Q/E(옆걸음), 손을 떼면 정지
- 지도·위치: 지도와 위치추정이 있으면 지도 위 로봇, "목표 지정"으로 Nav2 주행
- ⚠️ 인증이 없다. 같은 망의 누구나 조종할 수 있으므로 사내 시험망에서만 쓴다

## 원격 PC 에서 보기 (Foxglove, 보기 전용)

미니PC 는 부팅 시 `bpx-bridge.service` 가 `foxglove:=true` 로 실행 → `ws://<미니PC>:8765` 열림.
Windows 쪽에는 ROS 설치가 필요 없다.

1. Foxglove 앱 설치(foxglove.dev/download) 또는 브라우저로 app.foxglove.dev
2. Open connection → **Foxglove WebSocket** → `ws://192.168.30.52:8765`
   (WiFi DHCP 주소라 바뀔 수 있음 → 공유기에서 고정 할당 권장. `ws://undsw-PRO-Cubi-Z-AI-8M-MS-B032.local:8765` 도 가능)
3. 3D 패널: Fixed frame `odom`, URDF 는 `/robot_description` 에서 자동 로드 (첫 로드 때 메시 ~20 MB 전송)

보기 전용이다 (capabilities = connectionGraph, assets). 원격에서 /cmd_vel 발행·stand 호출 불가 — 조종은 미니PC SSH 에서.

## 로봇 없이 시험 (Windows, Docker Desktop)

```powershell
docker build -t bpx-ros -f docker/Dockerfile .
docker run --rm bpx-ros bash /bpx/docker/test_fake.sh     # 18개 항목 자동 시험
```

## 실기에서 확인해야 할 미확정 항목

- 미니PC 쪽 IP: 로봇이 DHCP 를 주는지 / 10.21.20.10 고정이 맞는지
- LegOdom 원점이 재연결·재부팅 때 리셋되는지 (map_to_odom 은 매 프레임 다시 계산하므로 영향 적음)
- IMU 장착 방향·frame (지금은 `base_link` 로 발행), 배터리 전류 단위/부호, 관절 토크 단위 (`effort` 비워둠)
- 관절 각 부호·영점이 URDF 와 일치하는지 (RViz 자세 vs 실물)
- 미니PC·라이다 전원: 후면 BAT Power Port(33V, 배터리 직결 추정)의 허용 전류·커넥터 규격 → 제조사 확인 후 DC-DC 로 공급할지, 별도 배터리를 쓸지 결정
- 탑재 무게 ≤ 5 kg

## 다음 단계 (미니PC·라이다 결정 후)

- 라이다 드라이버 + `base_link → lidar` static TF
- FAST-LIO 매핑 / NDT 위치추정 / Nav2 — Zishen 은 genisom 포크(`navigo_*`) 플러그인이라 그대로 못 씀.
  같은 미니PC(genisom 스택 설치됨)를 쓰면 재사용, 아니면 표준 Nav2 파라미터로 새로 작성
- 관제 브리지: `/bpx/motion` (stand|sit|damping), `/battery_state` 훅 준비돼 있음
