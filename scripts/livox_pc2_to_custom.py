#!/usr/bin/env python3
"""rosbag2 의 Livox PointCloud2(xfer_format 0) 를 FAST-LIO 용 CustomMsg(xfer_format 1) 로 변환.

  python3 scripts/livox_pc2_to_custom.py <입력 bag 폴더> <출력 bag 폴더> [--topic /livox/lidar]

입력 점 필드: x y z intensity(float32) tag line(uint8) timestamp(float64, 절대 ns)
  = livox_ros_driver2 가 xfer_format 0 으로 내는 형식.
라이다 토픽만 바꾸고 나머지(/livox/imu 등)는 그대로 복사한다.
점이 많아서 메시지 객체를 만들지 않고 numpy 로 CDR 바이트를 직접 붙인다 (첫 프레임은 표준 역직렬화로 검증).
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

from livox_ros_driver2.msg import CustomMsg

# CDR 에서 CustomPoint = uint32 + float32*3 + uint8*3 (19 바이트), 다음 원소의 uint32 정렬로 1 바이트 패딩
POINT_DT = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                     ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1'), ('pad', 'u1')])


def convert(pc: PointCloud2) -> bytes:
    names = {f.name: f for f in pc.fields}
    for need in ('x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'):
        if need not in names:
            raise ValueError(f"필드 '{need}' 없음 (xfer_format 0 형식이 아님): {list(names)}")
    n = pc.width * pc.height
    src = np.frombuffer(bytes(pc.data), dtype=np.dtype({
        'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
        'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
        'offsets': [names[k].offset for k in ('x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp')],
        'itemsize': pc.point_step}), count=n)

    ts = src['timestamp'].astype(np.uint64)
    timebase = int(ts.min()) if n else 0
    pts = np.zeros(n, dtype=POINT_DT)
    pts['offset_time'] = (ts - np.uint64(timebase)).astype(np.uint32)
    for k in ('x', 'y', 'z', 'tag', 'line'):
        pts[k] = src[k]
    pts['reflectivity'] = np.clip(src['intensity'], 0, 255).astype(np.uint8)

    head = CustomMsg()
    head.header = pc.header
    head.timebase = timebase
    head.point_num = n
    raw = serialize_message(head)          # points 길이 0 으로 직렬화 -> 끝 4 바이트가 시퀀스 길이
    body = pts.tobytes()[:-1] if n else b''  # 마지막 원소 뒤 패딩은 없음
    return raw[:-4] + np.uint32(n).tobytes() + body


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--topic', default='/livox/lidar')
    a = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=a.src, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=a.dst, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in reader.get_all_topics_and_types():
        typ = 'livox_ros_driver2/msg/CustomMsg' if t.name == a.topic else t.type
        writer.create_topic(rosbag2_py.TopicMetadata(name=t.name, type=typ, serialization_format='cdr'))

    frames = 0
    checked = False
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic == a.topic:
            pc = deserialize_message(data, PointCloud2)
            out = convert(pc)
            if not checked:   # 직접 만든 CDR 이 표준 역직렬화와 일치하는지 첫 프레임에서 확인
                msg = deserialize_message(out, CustomMsg)
                assert msg.point_num == len(msg.points) == pc.width * pc.height
                p = msg.points[-1]
                print(f"검증: points={msg.point_num} timebase={msg.timebase} "
                      f"마지막 점 offset={p.offset_time}ns xyz=({p.x:.3f},{p.y:.3f},{p.z:.3f}) line={p.line}")
                checked = True
            writer.write(topic, out, stamp)
            frames += 1
            if frames % 500 == 0:
                print(f"  {frames} 프레임", file=sys.stderr)
        else:
            writer.write(topic, data, stamp)
    print(f"완료: {frames} 프레임 변환 -> {a.dst}")


if __name__ == '__main__':
    main()
