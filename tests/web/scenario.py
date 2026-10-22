import json, time, websocket
ws = websocket.create_connection('ws://127.0.0.1:8090/ws', timeout=5)
S = {}
def pump(sec):
    end = time.time() + sec
    while time.time() < end:
        ws.settimeout(max(0.05, end - time.time()))
        try: m = json.loads(ws.recv())
        except Exception: continue
        if m['type'] == 'state': S.update(m)
def st(): return S['status']
def tele(vx, vy, wz, sec):
    end = time.time() + sec
    while time.time() < end:
        ws.send(json.dumps({'type': 'teleop', 'vx': vx, 'vy': vy, 'wz': wz})); pump(0.1)
ok = lambda c, msg: print(('[PASS] ' if c else '[FAIL] ') + msg)

pump(1.0)
ws.send(json.dumps({'type': 'action', 'name': 'stand'})); pump(0.8)
ok(st()['mode'] == 'idle' and any('allow_motion=false' in e['text'] for e in S['events']), '조종 모드 꺼짐이면 일어서기 거부')
ws.send(json.dumps({'type': 'motion', 'enable': True})); pump(0.8)
ok(st()['allow_motion'] is True, '조종 모드 켜기')
ws.send(json.dumps({'type': 'action', 'name': 'stand'})); pump(3.0)
ok(st()['mode'] == 'ready' and st()['motion_state'] == 6, f"일어서기 -> ready (mode={st()['mode']})")
x0 = S['pose'][1]; tele(0.3, 0, 0, 2.0)
ok(S['pose'][1] - x0 > 0.4, f"조이스틱 전진 0.3 m/s x2s -> {S['pose'][1]-x0:.2f} m 이동")
pump(1.0)
ok(abs(S['odom']['vx']) < 1e-3, f"조이스틱 입력 중단 -> 정지 (vx={S['odom']['vx']})")
tele(9.0, 0, 0, 1.0)
ok(abs(S['odom']['vx'] - 0.4) < 1e-3, f"과속 요청 9.0 -> 0.4 로 제한 (vx={S['odom']['vx']})")
tele(0.3, 0, 0, 0.5); ws.send(json.dumps({'type': 'motion', 'enable': False})); pump(1.0)
ok(st()['allow_motion'] is False and st()['mode'] == 'idle' and abs(S['odom']['vx']) < 1e-3, '주행 중 조종 모드 끄기 -> 정지')
ws.send(json.dumps({'type': 'motion', 'enable': True})); pump(0.5)
ws.send(json.dumps({'type': 'action', 'name': 'sit'})); pump(3.0)
ok(st()['motion_state'] == 0, f"앉기 -> 엎드림 (motion_state={st()['motion_state']})")
ws.send(json.dumps({'type': 'goal', 'x': 1.0, 'y': 0.0, 'yaw': 0.0})); pump(1.0)
ok(any('Nav2' in e['text'] for e in S['events']), '목표 지정: Nav2 미실행 안내')
# 주행 중 화면을 닫으면 정지
ws.send(json.dumps({'type': 'action', 'name': 'stand'})); pump(3.0)
tele(0.3, 0, 0, 0.5); ws.close()
ws = websocket.create_connection('ws://127.0.0.1:8090/ws', timeout=5); pump(1.5)
ok(abs(S['odom']['vx']) < 1e-3, f"주행 중 화면 닫힘 -> 정지 (vx={S['odom']['vx']})")
ws.send(json.dumps({'type': 'action', 'name': 'sit'})); pump(3.0)
ws.send(json.dumps({'type': 'motion', 'enable': False})); pump(0.5)
print('기록:'); [print('  ', e['t'], e['text']) for e in S['events'][-8:]]
