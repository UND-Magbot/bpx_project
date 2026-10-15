#!/usr/bin/env bash
# bpx-bridge.service 설치 + 부팅 시 자동 시작 (읽기 전용 브리지)
# 사용: sudo bash setup/install_service.sh   /   해제: sudo systemctl disable --now bpx-bridge
set -euo pipefail
cp "$(dirname "$0")/bpx-bridge.service" /etc/systemd/system/bpx-bridge.service
systemctl daemon-reload
systemctl enable --now bpx-bridge
systemctl --no-pager status bpx-bridge | head -5
