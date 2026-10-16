#!/usr/bin/env python3
"""FAST-LIO 지도(PCD)를 중력 방향으로 세운다.

  python3 scripts/align_map_gravity.py ~/bpx/maps/office.pcd                 # office.gravity.json 사용
  python3 scripts/align_map_gravity.py map.pcd --gravity 0.07 -0.10 0.99     # 값 직접 지정
  -> office_level.pcd (같은 폴더)

FAST-LIO camera_init = 출발 시 IMU 자세. 그때 잰 중력 벡터(IMU 좌표)를 +z 로 보내는 회전을 지도 전체에 곱한다.
yaw(수평 방향)와 원점은 그대로라 출발 위치 = 지도 원점 관계는 유지된다.
PCD 는 FAST-LIO 가 쓰는 binary float32 형식만 지원한다.
"""
import argparse
import json
import os
import sys

import numpy as np


def rot_to_z(g):
    """벡터 g 방향을 +z 로 보내는 최소 회전 (Rodrigues)."""
    u = np.asarray(g, float) / np.linalg.norm(g)
    v = np.cross(u, [0.0, 0.0, 1.0])
    s, c = np.linalg.norm(v), u[2]
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / s ** 2)


def read_pcd(path):
    raw = open(path, 'rb').read()
    i = raw.index(b'DATA ')
    end = raw.index(b'\n', i) + 1
    header = raw[:end].decode('ascii')
    if 'DATA binary\n' not in header:
        sys.exit("binary PCD 만 지원 (FAST-LIO 출력 형식)")
    fields = next(l for l in header.splitlines() if l.startswith('FIELDS')).split()[1:]
    types = next(l for l in header.splitlines() if l.startswith('TYPE')).split()[1:]
    sizes = next(l for l in header.splitlines() if l.startswith('SIZE')).split()[1:]
    if set(types) != {'F'} or set(sizes) != {'4'}:
        sys.exit("float32 필드만 지원")
    pts = np.frombuffer(raw[end:], dtype='<f4').reshape(-1, len(fields)).copy()
    return header, fields, pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--gravity', type=float, nargs=3, metavar=('GX', 'GY', 'GZ'))
    ap.add_argument('-o', '--output')
    a = ap.parse_args()

    if a.gravity:
        g = a.gravity
    else:
        js = os.path.splitext(a.pcd)[0] + '.gravity.json'
        if not os.path.exists(js):
            sys.exit(f"{js} 없음 -> --gravity 로 지정 (매핑 시작 1초 IMU 평균)")
        g = json.load(open(js))['gravity_imu']
    R = rot_to_z(g)
    tilt = np.degrees(np.arccos(np.clip(np.asarray(g)[2] / np.linalg.norm(g), -1, 1)))

    header, fields, pts = read_pcd(a.pcd)
    xyz = pts[:, :3].astype(np.float64)
    z_before = xyz[:, 2]
    pts[:, :3] = (xyz @ R.T).astype(np.float32)
    if {'normal_x', 'normal_y', 'normal_z'} <= set(fields):
        ni = [fields.index(f) for f in ('normal_x', 'normal_y', 'normal_z')]
        pts[:, ni] = (pts[:, ni].astype(np.float64) @ R.T).astype(np.float32)

    out = a.output or os.path.splitext(a.pcd)[0] + '_level.pcd'
    with open(out, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.astype('<f4').tobytes())
    zb, za = np.percentile(z_before, [1, 99]), np.percentile(pts[:, 2], [1, 99])
    print(f"기울기 {tilt:.2f}° 보정, 점 {len(pts)}개")
    print(f"z 범위(1~99%): {zb[0]:.1f}~{zb[1]:.1f} m -> {za[0]:.1f}~{za[1]:.1f} m")
    print(f"저장: {out}")


if __name__ == '__main__':
    main()
