"""관리자 웹 설정 (~/bpx/config/web_settings.json). 매핑·위치추정 세션과 2D 지도 생성이 읽는다.

속도 상한은 여기 저장하지 않는다: 브리지를 다시 켜면 bridge.yaml 기본값(보수적)으로 돌아가는 것이 안전하다.
"""
import json
import os

PATH = os.path.expanduser('~/bpx/config/web_settings.json')
DEFAULTS = {
    # Livox Mid-360S: IP 는 192.168.1.1XX (XX = SN 끝 두 자리). 장착 위치·자세 = base_link -> livox_frame
    'lidar': {'ip': '192.168.1.100', 'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    # Nav2 2D 지도 생성 (지면 기준 장애물 높이 범위)
    'occ': {'resolution': 0.05, 'obstacle_min': 0.3, 'obstacle_max': 2.0},
}
LIMITS = {
    'lidar': {'x': (-1, 1), 'y': (-1, 1), 'z': (-0.5, 1.5), 'roll': (-3.1416, 3.1416),
              'pitch': (-1.6, 1.6), 'yaw': (-3.1416, 3.1416)},
    'occ': {'resolution': (0.02, 0.5), 'obstacle_min': (0.05, 1.5), 'obstacle_max': (0.3, 5.0)},
}


def load():
    cfg = json.loads(json.dumps(DEFAULTS))
    try:
        saved = json.load(open(PATH))
        for sec in cfg:
            cfg[sec].update({k: v for k, v in saved.get(sec, {}).items() if k in cfg[sec]})
    except (OSError, ValueError):
        pass
    return cfg


def save(update):
    """update = {'lidar': {...}, 'occ': {...}} 일부만 와도 된다. 범위를 벗어나면 ValueError."""
    cfg = load()
    for sec, vals in update.items():
        if sec not in cfg:
            raise ValueError(f"알 수 없는 설정 항목: {sec}")
        for k, v in vals.items():
            if k not in cfg[sec]:
                raise ValueError(f"알 수 없는 설정: {sec}.{k}")
            if k == 'ip':
                parts = str(v).split('.')
                if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
                    raise ValueError(f"IP 형식 오류: {v}")
                cfg[sec][k] = str(v)
                continue
            v = float(v)
            lo, hi = LIMITS[sec][k]
            if not lo <= v <= hi:
                raise ValueError(f"{sec}.{k} 허용 범위 {lo}~{hi}")
            cfg[sec][k] = v
    if cfg['occ']['obstacle_min'] >= cfg['occ']['obstacle_max']:
        raise ValueError('장애물 높이: 최소가 최대보다 작아야 함')
    os.makedirs(os.path.dirname(PATH), exist_ok=True)
    with open(PATH, 'w') as f:
        json.dump(cfg, f, indent=2)
    return cfg


def lidar_launch_args():
    lid = load()['lidar']
    return [f"lidar_ip:={lid['ip']}"] + [f"lidar_{k}:={lid[k]}" for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]


def occ_args():
    o = load()['occ']
    return ['--resolution', str(o['resolution']), '--obstacle-height-m', str(o['obstacle_min']),
            '--max-obstacle-height-m', str(o['obstacle_max']), '--inflate-radius-m', '0', '--padding-m', '2.0',
            '--min-points-per-cell', '2']
