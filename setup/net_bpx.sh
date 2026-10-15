#!/usr/bin/env bash
# 미니PC 유선 NIC 고정 IP (NetworkManager 용 — 이 미니PC는 netplan renderer=NetworkManager)
#   로봇 NIC  : 10.21.20.10/24  (BPX RJ45 직결, 로봇 = 10.21.20.1)
#   라이다 NIC: 192.168.1.50/24 (Mid-360 기본 대역)
# 둘 다 게이트웨이/기본경로 없음 -> 인터넷은 WiFi 유지.
# 사용: sudo bash setup/net_bpx.sh [로봇NIC=eno1] [라이다NIC=enp4s0]
# 랜선을 안 꽂아도 설정은 저장되고, 꽂는 순간 자동으로 올라온다.
set -euo pipefail
ROBOT_NIC="${1:-eno1}"
LIDAR_NIC="${2:-enp4s0}"

add_static() {   # 연결이름 NIC 주소
  local name=$1 nic=$2 addr=$3
  nmcli -g NAME con show | grep -qxF "$name" && nmcli con delete "$name" >/dev/null
  nmcli con add type ethernet con-name "$name" ifname "$nic" \
    ipv4.method manual ipv4.addresses "$addr" ipv4.never-default yes \
    ipv6.method disabled connection.autoconnect yes >/dev/null
  echo "  $name: $nic -> $addr"
}

echo "[BPX] 유선 NIC 고정 IP"
add_static bpx-robot "$ROBOT_NIC" 10.21.20.10/24
add_static bpx-lidar "$LIDAR_NIC" 192.168.1.50/24
nmcli -f NAME,DEVICE,TYPE con show | grep -E 'NAME|bpx-'
echo "로봇 랜선을 $ROBOT_NIC 에 꽂은 뒤: ping 10.21.20.1"
