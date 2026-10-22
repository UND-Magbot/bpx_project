"""지도 저장소 (~/bpx/maps/<이름>/). 관리자 웹 지도 탭.

폴더 구성: map.pcd(FAST-LIO 원본) gravity.json(매핑 시작 중력) map_level.pcd(수평 보정, 위치추정용)
           map.yaml/map.pgm(Nav2 2D 지도) preview.png(미리보기) meta.json(정보)
"""
import json
import os
import re
import shutil
import subprocess
import threading
import time

from bpx_web import settings

MAP_ROOT = os.path.expanduser('~/bpx/maps')
ALIGN = os.path.expanduser('~/bpx/scripts/align_map_gravity.py')
ACTIVE = os.path.join(MAP_ROOT, 'active.json')
NAME_RE = re.compile(r'^[\w\-]{1,40}$')      # 영문·숫자·한글·_·-


def pool_grid(grid, k, occ, free, unknown):
    """k x k 블록으로 줄이되 평균하지 않고 우선순위로: 장애물 > 빈 공간 > 미확인 (얇은 벽이 사라지지 않게)."""
    import numpy as np
    h, w = grid.shape
    H, W = -(-h // k), -(-w // k)
    pad = np.full((H * k, W * k), unknown, dtype=grid.dtype)
    pad[:h, :w] = grid
    b = pad.reshape(H, k, W, k)
    out = np.full((H, W), unknown, dtype=grid.dtype)
    out[(b == free).any(axis=(1, 3))] = free
    out[(b == occ).any(axis=(1, 3))] = occ
    return out


def _pcd_points(path):
    try:
        with open(path, 'rb') as f:
            for _ in range(15):
                line = f.readline().decode('ascii', 'ignore')
                if line.startswith('POINTS'):
                    return int(line.split()[1])
    except OSError:
        pass
    return None


class MapStore:
    def __init__(self, log=print):
        os.makedirs(MAP_ROOT, exist_ok=True)
        self.log = log
        self.jobs = {}                      # name -> {'state','step','error'}
        self.lock = threading.Lock()

    # ---------------- 조회 ----------------
    def path(self, name, *parts):
        if not NAME_RE.match(name or ''):
            raise ValueError(f"지도 이름 형식 오류: {name!r} (영문·숫자·한글·_·- 40자 이내)")
        return os.path.join(MAP_ROOT, name, *parts)

    def active(self):
        try:
            return json.load(open(ACTIVE)).get('name')
        except (OSError, ValueError):
            return None

    def meta(self, name):
        try:
            return json.load(open(self.path(name, 'meta.json')))
        except (OSError, ValueError):
            return {}

    def info(self, name):
        p = lambda f: self.path(name, f)
        files = {f: os.path.exists(p(f)) for f in ('map.pcd', 'gravity.json', 'map_level.pcd', 'map.yaml')}
        size = sum(os.path.getsize(os.path.join(p(''), f)) for f in os.listdir(p('')) if os.path.isfile(os.path.join(p(''), f)))
        m = self.meta(name)
        grav = {}
        if files['gravity.json']:
            try:
                grav = json.load(open(p('gravity.json')))
            except ValueError:
                pass
        with self.lock:
            job = dict(self.jobs.get(name, {}))
        ready = files['map_level.pcd'] and files['map.yaml']
        return {
            'name': name, 'created': m.get('created') or time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(p('')))),
            'source': m.get('source', ''), 'points': _pcd_points(p('map.pcd')), 'size_mb': round(size / 1e6, 1),
            'files': files, 'tilt_deg': round(grav['tilt_deg'], 1) if 'tilt_deg' in grav else None,
            'grid': m.get('grid'), 'ready': ready, 'active': self.active() == name, 'job': job,
            'note': m.get('note', ''),
        }

    def list(self):
        out = []
        for d in sorted(os.listdir(MAP_ROOT)):
            if NAME_RE.match(d) and os.path.isdir(os.path.join(MAP_ROOT, d)):
                try:
                    out.append(self.info(d))
                except OSError:
                    continue
        return sorted(out, key=lambda m: m['created'], reverse=True)

    def preview_png(self, name):
        png = self.path(name, 'preview.png')
        pgm = self.path(name, 'map.pgm')
        if not os.path.exists(pgm):
            return None
        if not os.path.exists(png) or os.path.getmtime(png) < os.path.getmtime(pgm):
            import numpy as np
            from PIL import Image
            g = np.array(Image.open(pgm))                 # trinary pgm: 0 장애물, 254 빈 공간, 205 미확인
            k = max(1, -(-max(g.shape) // 900))
            if k > 1:
                g = pool_grid(g, k, 0, 254, 205)
            Image.fromarray(g).save(png)
        with open(png, 'rb') as f:
            return f.read()

    # ---------------- 변경 ----------------
    def write_meta(self, name, **kv):
        m = self.meta(name)
        m.update(kv)
        with open(self.path(name, 'meta.json'), 'w') as f:
            json.dump(m, f, ensure_ascii=False, indent=2)

    def set_active(self, name):
        if name is not None and not self.info(name)['ready']:
            raise ValueError('처리(수평 보정·2D 지도)가 끝난 지도만 활성으로 지정할 수 있음')
        with open(ACTIVE, 'w') as f:
            json.dump({'name': name}, f)

    def rename(self, old, new):
        src, dst = self.path(old), self.path(new)
        if os.path.exists(dst):
            raise ValueError(f"같은 이름의 지도가 이미 있음: {new}")
        os.rename(src, dst)
        yaml = os.path.join(dst, 'map.yaml')
        if self.active() == old:
            self.set_active(new)
        return os.path.exists(yaml)

    def delete(self, name, busy=False):
        if busy:
            raise ValueError('이 지도를 쓰는 세션이 실행 중')
        if self.active() == name:
            raise ValueError('활성 지도는 삭제할 수 없음 (다른 지도를 활성으로 지정한 뒤 삭제)')
        shutil.rmtree(self.path(name))

    # ---------------- 처리 (저장 후 자동) ----------------
    def process_async(self, name, done=None):
        with self.lock:
            if self.jobs.get(name, {}).get('state') == 'running':
                raise ValueError('이미 처리 중')
            self.jobs[name] = {'state': 'running', 'step': '대기', 'error': ''}
        threading.Thread(target=self._process, args=(name, done), daemon=True).start()

    def _step(self, name, step):
        with self.lock:
            self.jobs[name]['step'] = step
        self.log(f"[지도 {name}] {step}")

    def _process(self, name, done):
        try:
            pcd = self.path(name, 'map.pcd')
            if not os.path.exists(pcd):
                raise RuntimeError('map.pcd 없음 (저장이 안 됨)')
            self._step(name, '1/2 중력 방향으로 수평 보정')
            if not os.path.exists(self.path(name, 'gravity.json')):
                raise RuntimeError('gravity.json 없음 (매핑 시작 때 IMU 기록 실패)')
            r = subprocess.run(['python3', ALIGN, pcd], capture_output=True, text=True, timeout=300)
            if r.returncode:
                raise RuntimeError('수평 보정 실패: ' + (r.stderr or r.stdout)[-300:])
            self._step(name, '2/2 Nav2 2D 지도 생성')
            r = subprocess.run(['ros2', 'run', 'lidar_localization_ros2', 'generate_occupancy_map_from_pcd',
                                '--pcd', self.path(name, 'map_level.pcd'), '--output-dir', self.path(name),
                                '--map-name', 'map'] + settings.occ_args(), capture_output=True, text=True, timeout=600)
            if r.returncode or not os.path.exists(self.path(name, 'map.yaml')):
                raise RuntimeError('2D 지도 생성 실패: ' + (r.stderr or r.stdout)[-300:])
            grid = None
            try:
                from PIL import Image
                w, h = Image.open(self.path(name, 'map.pgm')).size
                res = settings.load()['occ']['resolution']
                grid = {'w': w, 'h': h, 'res': res, 'm': [round(w * res, 1), round(h * res, 1)]}
            except Exception:
                pass
            self.write_meta(name, processed=time.strftime('%Y-%m-%d %H:%M'), grid=grid)
            with self.lock:
                self.jobs[name] = {'state': 'done', 'step': '완료', 'error': ''}
            self.log(f"[지도 {name}] 처리 완료")
        except Exception as e:
            with self.lock:
                self.jobs[name] = {'state': 'error', 'step': '실패', 'error': str(e)}
            self.log(f"[지도 {name}] 처리 실패: {e}")
        if done:
            done(name)
