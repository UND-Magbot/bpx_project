"""미니PC 시스템 상태 (관리자 웹 시스템 탭). 외부 패키지 없이 /proc, /sys 만 읽는다."""
import glob
import os
import shutil
import socket
import subprocess
import time

NICS = {'eno1': '로봇', 'enp4s0': '라이다', 'wlp3s0': 'WiFi'}
SERVICES = ('bpx-bridge',)


class SysInfo:
    def __init__(self):
        self._cpu_prev = self._cpu_times()

    @staticmethod
    def _cpu_times():
        with open('/proc/stat') as f:
            v = [int(x) for x in f.readline().split()[1:]]
        idle = v[3] + v[4]
        return sum(v), idle

    def cpu_percent(self):
        total, idle = self._cpu_times()
        pt, pi = self._cpu_prev
        self._cpu_prev = (total, idle)
        dt = total - pt
        return round(100.0 * (1 - (idle - pi) / dt), 1) if dt > 0 else 0.0

    @staticmethod
    def memory():
        info = {}
        with open('/proc/meminfo') as f:
            for line in f:
                k, v = line.split(':')
                info[k] = int(v.split()[0])
        total, avail = info['MemTotal'], info['MemAvailable']
        return {'total_gb': round(total / 1048576, 1), 'used_gb': round((total - avail) / 1048576, 1),
                'percent': round(100.0 * (total - avail) / total, 1)}

    @staticmethod
    def temperatures():
        out = {}
        for d in glob.glob('/sys/class/hwmon/hwmon*'):
            try:
                name = open(os.path.join(d, 'name')).read().strip()
                t = int(open(os.path.join(d, 'temp1_input')).read()) / 1000.0
            except (OSError, ValueError):
                continue
            out[name] = round(t, 1)
        return out

    @staticmethod
    def nics():
        res = []
        for nic, role in NICS.items():
            base = f'/sys/class/net/{nic}'
            if not os.path.isdir(base):
                continue
            try:
                state = open(f'{base}/operstate').read().strip()
                carrier = open(f'{base}/carrier').read().strip() == '1'
            except OSError:
                state, carrier = 'unknown', False
            ip = subprocess.run(['ip', '-4', '-o', 'addr', 'show', nic], capture_output=True, text=True).stdout.split()
            addr = ip[ip.index('inet') + 1] if 'inet' in ip else ''
            res.append({'nic': nic, 'role': role, 'state': state, 'link': carrier, 'addr': addr})
        return res

    @staticmethod
    def services():
        out = {}
        for s in SERVICES:
            r = subprocess.run(['systemctl', 'is-active', s], capture_output=True, text=True)
            out[s] = r.stdout.strip()
        return out

    def collect(self):
        du = shutil.disk_usage('/')
        with open('/proc/uptime') as f:
            up = float(f.read().split()[0])
        return {
            'host': socket.gethostname(),
            'time': time.strftime('%Y-%m-%d %H:%M:%S'),
            'uptime_h': round(up / 3600, 1),
            'cpu_percent': self.cpu_percent(),
            'load': [round(x, 2) for x in os.getloadavg()],
            'cpu_count': os.cpu_count(),
            'memory': self.memory(),
            'disk': {'total_gb': round(du.total / 1e9), 'used_gb': round(du.used / 1e9), 'percent': round(100 * du.used / du.total, 1)},
            'temps': self.temperatures(),
            'nics': self.nics(),
            'services': self.services(),
        }
