# 관리자 웹 시나리오 시험 (fake 로봇)

실제 로봇 없이 fake 브리지 + 관리자 웹(포트 8090)을 **ROS_DOMAIN_ID=93** 에 띄워 웹 기능을 시험한다.
실제 서비스(도메인 90, 포트 8080)와 섞이지 않는다.

```bash
# 1) fake 브리지 + 웹 시작 (세션이 끊겨도 유지되게 systemd-run 권장)
sudo systemd-run --unit=bpx-webtest --uid=$USER -E HOME=$HOME -p KillMode=control-group -p MemoryMax=8G \
  /bin/bash ~/bpx/tests/web/run.sh
sleep 7

# 2) 시나리오 (각각 독립 실행)
python3 -I ~/bpx/tests/web/scenario.py        # 조종: 조종 모드·서기·조이스틱·속도 제한·정지 (10항목)
python3 -I ~/bpx/tests/web/check_scenario.py  # 점검 탭: 주행 점검·거리·전력·배터리 (7항목)
python3 -I ~/bpx/tests/web/dev_scenario.py    # 설정·속도 상한·Nav2 시험 (9항목)
python3 -I ~/bpx/tests/web/map_scenario.py    # 매핑·지도 관리·위치추정 (15항목, 공개 bag 필요, 약 5분)
python3 -I ~/bpx/tests/web/imu_wiz.py         # IMU 점검 판정 (헤드리스 Firefox + geckodriver)

# 3) 정리
sudo systemctl stop bpx-webtest
```

- map_scenario 는 `~/bpx/lidar_ws/bags/` 의 공개 bag 이 필요하다 (개발문서 "시험 데이터" 참고).
- dev_scenario 는 `~/bpx/config/web_settings.json` 을 바꾸므로 끝나면 원래대로 돌려놓을 것.
- 로그: `~/bpx/lidar_ws/webtest/bridge.log`, `web.log`
