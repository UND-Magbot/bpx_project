"""세션 프로세스 관리 (매핑 / 위치추정). 라이다 드라이버 출력 형식이 달라 동시에 하나만 돈다."""
import os
import signal
import subprocess
import threading
import time
from collections import deque

LOG_DIR = os.path.expanduser('~/.ros/bpx_web_sessions')


class Session:
    def __init__(self, kind, cmd, info):
        os.makedirs(LOG_DIR, exist_ok=True)
        self.kind, self.cmd, self.info = kind, cmd, info
        self.started = time.time()
        self.log_path = os.path.join(LOG_DIR, f"{kind}_{time.strftime('%Y%m%d_%H%M%S')}.log")
        self.tail = deque(maxlen=40)
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     start_new_session=True, bufsize=1)
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        with open(self.log_path, 'w') as f:
            for line in self.proc.stdout:
                f.write(line)
                f.flush()
                if 'rosbag2_player' in line and ('Press ' in line or 'keyboard' in line.lower()):
                    continue                     # bag 재생기 키보드 안내 문구는 화면 로그에서 뺀다
                self.tail.append(line.rstrip()[-200:])

    def alive(self):
        return self.proc.poll() is None

    def stop(self, timeout=12.0):
        """SIGINT -> (ros2 launch 가 자식 정리) -> 남으면 SIGTERM -> SIGKILL.
        라이다 미연결 상태의 Livox 드라이버는 SIGINT/SIGTERM 을 무시해서 마지막 단계가 필요하다."""
        if not self.alive():
            return
        pg = os.getpgid(self.proc.pid)
        for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(pg, sig)
            except ProcessLookupError:
                return
            try:
                self.proc.wait(timeout=wait)
                if sig == signal.SIGINT:
                    time.sleep(0.5)          # launch 가 끝난 뒤에도 남은 자식이 있으면 다음 단계에서 정리
                    os.killpg(pg, 0)         # 그룹이 살아 있으면 계속
                else:
                    return
            except ProcessLookupError:
                return
            except subprocess.TimeoutExpired:
                continue

    def status(self):
        return {'kind': self.kind, 'alive': self.alive(), 'started': self.started,
                'elapsed': round(time.time() - self.started), 'info': self.info,
                'returncode': self.proc.poll(), 'log': list(self.tail)[-12:], 'log_path': self.log_path}


class SessionManager:
    def __init__(self):
        self.cur = None
        self.lock = threading.Lock()

    def start(self, kind, cmd, info):
        with self.lock:
            if self.cur and self.cur.alive():
                raise RuntimeError(f"이미 {self.cur.kind} 세션이 실행 중")
            self.cur = Session(kind, cmd, info)
            return self.cur

    def stop(self):
        with self.lock:
            s = self.cur
        if s:
            s.stop()
        return s

    def status(self):
        with self.lock:
            return self.cur.status() if self.cur else None

    def running(self, kind=None):
        with self.lock:
            return bool(self.cur and self.cur.alive() and (kind is None or self.cur.kind == kind))
