import json, time, math, os, websocket
ws = websocket.create_connection('ws://127.0.0.1:8090/ws', timeout=5); S = {}; SYS = {}
def pump(s):
    end = time.time() + s
    while time.time() < end:
        ws.settimeout(max(0.05, end - time.time()))
        try: m = json.loads(ws.recv())
        except Exception: continue
        if m['type'] == 'state': S.update(m)
        elif m['type'] == 'system': SYS.update(m)
ev = lambda n=4: [e['text'] for e in S['events'][-n:]]
ok = lambda c, m: print(('[PASS] ' if c else '[FAIL] ') + m, flush=True)
pump(2.5)
ok(S['status'].get('limits') == [0.4, 0.25, 1.0, 0.5], f"기본 상한 {S['status'].get('limits')}")
ws.send(json.dumps({'type': 'set_limits', 'vx': 0.6, 'vy': 0.3, 'wz': 1.2, 'timeout': 0.5})); pump(1)
ok(S['status']['limits'][:3] == [0.6, 0.3, 1.2] and list(S['max_v']) == [0.6, 0.3, 1.2], f"상한 변경 → 브리지 {S['status']['limits']} / 웹 {S['max_v']}")
ws.send(json.dumps({'type': 'set_limits', 'vx': 3.0, 'vy': 0.3, 'wz': 1.2, 'timeout': 0.5})); pump(1)
ok(any('거부' in e for e in ev()) and S['status']['limits'][0] == 0.6, 'SDK 상한 초과(3.0) 거부')
ws.send(json.dumps({'type': 'motion', 'enable': True})); pump(0.5); ws.send(json.dumps({'type': 'action', 'name': 'stand'})); pump(3)
end = time.time() + 1.5
while time.time() < end: ws.send(json.dumps({'type': 'teleop', 'vx': 9, 'vy': 0, 'wz': 0})); pump(0.1)
ok(abs(S['odom']['vx'] - 0.6) < 1e-3, f"조이스틱 과속 → 새 상한 0.6 에서 잘림 (vx={S['odom']['vx']})")
ws.send(json.dumps({'type': 'teleop', 'vx': 0, 'vy': 0, 'wz': 0})); pump(1)
ws.send(json.dumps({'type': 'save_settings', 'settings': {'lidar': {'ip': '192.168.1.157', 'roll': 3.14159, 'z': 0.3}}})); pump(2.5)
ok(SYS.get('settings', {}).get('lidar', {}).get('ip') == '192.168.1.157', f"라이다 설정 저장 ({SYS.get('settings', {}).get('lidar')})")
ws.send(json.dumps({'type': 'save_settings', 'settings': {'lidar': {'ip': '192.168.1.999'}}})); pump(0.6)
ok(any('IP 형식' in e for e in ev()), '잘못된 IP 거부')
ws.send(json.dumps({'type': 'nav_test_start'})); pump(14)
ok(S['session'] and S['session']['kind'] == 'nav_test' and S['session']['alive'], 'Nav2 시험 세션 실행')
p0 = S['pose']; gx, gy = p0[1] + 1.5, p0[2] + 0.5
ws.send(json.dumps({'type': 'goal', 'x': gx, 'y': gy, 'yaw': 0.0})); pump(1)
for _ in range(30):
    if S['goal_state'] in ('도착', '실패', '취소됨', '거부됨'): break
    pump(1)
err = math.hypot(S['pose'][1] - gx, S['pose'][2] - gy)
ok(S['goal_state'] == '도착' and err < 0.25, f"Nav2 목표 (1.5, 0.5) → {S['goal_state']}, 오차 {err:.2f} m, 위치 {[round(v, 2) for v in S['pose'][1:3]]}")
ws.send(json.dumps({'type': 'session_stop'})); pump(10)
ok(not (S['session'] and S['session']['alive']), 'Nav2 시험 정지')
ws.send(json.dumps({'type': 'set_limits', 'vx': 0.4, 'vy': 0.25, 'wz': 1.0, 'timeout': 0.5})); pump(0.6)
ws.send(json.dumps({'type': 'action', 'name': 'sit'})); pump(3); ws.send(json.dumps({'type': 'motion', 'enable': False})); pump(0.3)
