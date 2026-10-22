import json, time, websocket
ws = websocket.create_connection('ws://127.0.0.1:8090/ws', timeout=5)
S, MAPS, MP = {}, [], {'cells': 0, 'path': 0}
def pump(s):
    end = time.time() + s
    while time.time() < end:
        ws.settimeout(max(0.05, end - time.time()))
        try: m = json.loads(ws.recv())
        except Exception: continue
        if m['type'] == 'state': S.update(m)
        elif m['type'] == 'maps': MAPS[:] = m['maps']
        elif m['type'] == 'mapping': MP['cells'] = m['total']; MP['path'] = len(m['path'])
def ev(n=6): return [e['text'] for e in S['events'][-n:]]
def mp(name): return next((m for m in MAPS if m['name'] == name), None)
ok = lambda c, m: print(('[PASS] ' if c else '[FAIL] ') + m, flush=True)
NAME = 'webtest_map'
pump(3)
ok(mp('aist_test') and mp('aist_test')['ready'], '지도 목록: aist_test 사용 가능')
ws.send(json.dumps({'type': 'mapping_start', 'name': 'bad name!', 'source': 'bag'})); pump(0.6)
ok(any('이름 형식' in e for e in ev(3)), '잘못된 지도 이름 거부')
ws.send(json.dumps({'type': 'mapping_start', 'name': NAME, 'source': 'bag'})); pump(40)
ok(S['session'] and S['session']['kind'] == 'mapping' and S['session']['alive'], '매핑 세션 실행 중')
ok(MP['cells'] > 500 and MP['path'] > 10, f"실시간 미리보기: 칸 {MP['cells']}개, 궤적 점 {MP['path']}개")
ws.send(json.dumps({'type': 'mapping_save'})); pump(25)
m = mp(NAME)
ok(m and m['files']['map.pcd'] and m['files']['gravity.json'], f"저장: map.pcd·gravity.json ({m and m['files']})")
for _ in range(20):
    if m and m['job'].get('state') in ('done', 'error'): break
    pump(2); m = mp(NAME)
ok(m and m['ready'] and m['job']['state'] == 'done', f"자동 처리 완료 (기울기 {m and m['tilt_deg']}°, 2D {m and m['grid']})")
ok(not (S['session'] and S['session']['alive']), '저장 후 세션 종료됨')
ws.send(json.dumps({'type': 'map_activate', 'name': 'aist_test'})); pump(2.5)
ok(S['active_map'] == 'aist_test', '활성 지도 지정')
ws.send(json.dumps({'type': 'loc_start', 'source': 'bag', 'nav': False})); pump(8)
p0 = S['pose']; pump(22)
ok(S['session'] and S['session']['kind'] == 'localization' and S['session']['alive'], '위치추정 세션 실행 중')
import math
moved = math.hypot(S['pose'][1] - p0[1], S['pose'][2] - p0[2]) if S['pose'] and p0 else 0
ok(S['pose'] and S['pose'][0] == 'map' and moved > 10, f"지도 좌표 위치가 따라 움직임 ({moved:.1f} m, {S['pose']})")
ok(S['map_version'] > 0, f"2D 지도 수신 (map_version={S['map_version']})")
ws.send(json.dumps({'type': 'session_stop'})); pump(15)
ok(not (S['session'] and S['session']['alive']), '위치추정 세션 정지')
ws.send(json.dumps({'type': 'map_delete', 'name': 'aist_test'})); pump(0.6)
ok(any('활성 지도는 삭제' in e for e in ev(3)), '활성 지도 삭제 거부')
ws.send(json.dumps({'type': 'map_rename', 'old': NAME, 'new': NAME + '2'})); pump(2.5)
ok(mp(NAME + '2') and not mp(NAME), '이름 변경')
ws.send(json.dumps({'type': 'map_delete', 'name': NAME + '2'})); pump(2.5)
ok(not mp(NAME + '2'), '지도 삭제')
print('기록:'); [print('  ', e['t'], e['text']) for e in S['events'][-10:]]
