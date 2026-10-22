import json, time, websocket
ws = websocket.create_connection('ws://127.0.0.1:8090/ws', timeout=5); S = {}; kinds = set()
def pump(s):
    end = time.time() + s
    while time.time() < end:
        ws.settimeout(max(0.05, end - time.time()))
        try: m = json.loads(ws.recv())
        except Exception: continue
        kinds.add(m['type'])
        if m['type'] == 'state': S.update(m)
ok = lambda c, m: print(('[PASS] ' if c else '[FAIL] ') + m, flush=True)
def hold(name, v, sec):
    ws.send(json.dumps({'type': 'test_begin', 'name': name}))
    end = time.time() + sec
    while time.time() < end: ws.send(json.dumps({'type': 'teleop', 'vx': v[0], 'vy': v[1], 'wz': v[2]})); pump(0.1)
    ws.send(json.dumps({'type': 'teleop', 'vx': 0, 'vy': 0, 'wz': 0})); pump(0.6)
    ws.send(json.dumps({'type': 'test_end'})); pump(0.6)
    return S['test_results'][-1]
pump(1); ws.send(json.dumps({'type': 'motion', 'enable': True})); pump(0.5); ws.send(json.dumps({'type': 'action', 'name': 'stand'})); pump(3)
for name, v in (('fwd', (0.2, 0, 0)), ('left', (0, 0.15, 0)), ('ccw', (0, 0, 0.5)), ('cw', (0, 0, -0.5))):
    r = hold(name, v, 2.5)
    ok(r['direction_ok'] and 80 <= r.get('ratio_pct', 0) <= 120, f"{r['label']}: 방향 {r['direction_ok']}, 측정 {r.get('measured')} / 명령 {r['want']} ({r.get('ratio_pct')}%), 지연 {r.get('latency_s')}s, 이동 앞{r['fwd_m']} 좌{r['left_m']} {r['yaw_deg']}°, {r.get('current_avg_a')}A")
ws.send(json.dumps({'type': 'test_begin', 'name': 'distance'})); pump(0.3)
end = time.time() + 3
while time.time() < end: ws.send(json.dumps({'type': 'teleop', 'vx': 0.2, 'vy': 0, 'wz': 0})); pump(0.1)
ws.send(json.dumps({'type': 'teleop', 'vx': 0, 'vy': 0, 'wz': 0})); pump(0.8)
ws.send(json.dumps({'type': 'test_end', 'actual_m': 0.6})); pump(0.6); r = S['test_results'][-1]
ok(r.get('odom_error_pct') is not None, f"거리 측정: odom {r.get('dist_m')} m / 실제 0.6 m → 오차 {r.get('odom_error_pct')}%")
ws.send(json.dumps({'type': 'test_begin', 'name': 'power_idle'})); pump(3); ws.send(json.dumps({'type': 'test_end'})); pump(0.6); r = S['test_results'][-1]
ok(r.get('current_avg_a') is not None, f"서기 대기 전력: {r.get('current_avg_a')} A · {r.get('power_w')} W")
pump(10.5)
ok('battery' in kinds, '배터리 장기 기록 메시지 수신')
ws.send(json.dumps({'type': 'action', 'name': 'sit'})); pump(3); ws.send(json.dumps({'type': 'motion', 'enable': False})); pump(0.3)
