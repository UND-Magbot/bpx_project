import json, subprocess, time, urllib.request, base64
def wd(m, p, b=None):
    r = urllib.request.Request('http://127.0.0.1:4444' + p, method=m, data=json.dumps(b).encode() if b is not None else None, headers={'Content-Type': 'application/json'})
    return json.load(urllib.request.urlopen(r, timeout=60))['value']
FEED_OLD = """
const seq = arguments[0]; const real = S.imu;
document.getElementById('imu-start').click();
for (const v of seq) { S.imu = {roll: v[0], pitch: v[1], yaw: 0, ax: v[2], ay: v[3], az: 9.8}; imuTick(); }
S.imu = real; return document.getElementById('imu-result').innerText;"""
GOOD = [[0, 0, 0, 0], [0, -8, 1.3, 0], [0, -9, 1.5, 0], [0, 0.5, 0, 0], [8, 0, 0, 1.3], [9, 0, 0, 1.5], [0.3, 0, 0, 0]]
BAD = [[0, 0, 0, 0], [0, 8, -1.3, 0], [0, 0.5, 0, 0], [-8, 0, 0, -1.3], [0.3, 0, 0, 0]]
gd = subprocess.Popen(['geckodriver', '--port', '4444'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(2)
try:
    sid = wd('POST', '/session', {'capabilities': {'alwaysMatch': {'moz:firefoxOptions': {'args': ['-headless', '--width=1500', '--height=1000']}}}})['sessionId']
    wd('POST', f'/session/{sid}/url', {'url': 'http://127.0.0.1:8090/'}); time.sleep(3)
    wd('POST', f'/session/{sid}/execute/sync', {'script': "document.querySelector('[data-view=check]').click(); return 1", 'args': []}); time.sleep(1)
    print('== 정상 축\n' + wd('POST', f'/session/{sid}/execute/sync', {'script': 'return bpxTest.imuFeed(arguments[0])', 'args': [GOOD]}))
    print('== 뒤집힌 축\n' + wd('POST', f'/session/{sid}/execute/sync', {'script': 'return bpxTest.imuFeed(arguments[0])', 'args': [BAD]}))
    time.sleep(11)
    open('/home/undsw/bpx/lidar_ws/webtest/tab_check.png', 'wb').write(base64.b64decode(wd('GET', f'/session/{sid}/screenshot')))
    wd('DELETE', f'/session/{sid}')
finally: gd.terminate()
