#!/usr/bin/env python3
# map->odom TF 발행기 (Zishen_ROS/map_to_odom.py 이식, 토픽/프레임만 파라미터화)
#  T_map_odom = T_map_base (localization) * inverse(T_odom_base (BPX LegOdom))
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s; y = (R[0, 2] - R[2, 0]) / s; z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s; y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s; y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s; y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return x, y, z, w


def pose_to_T(p):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
    T[0, 3], T[1, 3], T[2, 3] = p.position.x, p.position.y, p.position.z
    return T


class M2O(Node):
    def __init__(self):
        super().__init__('map_to_odom')
        loc_topic = self.declare_parameter('localization_topic', '/odom/localization_odom').value
        odom_topic = self.declare_parameter('odom_topic', '/odom/mc_odom').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.T_mb = None; self.T_ob = None
        self.create_subscription(Odometry, loc_topic, self.loc_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.mc_cb, 10)
        self.br = TransformBroadcaster(self)
        self.create_timer(0.03, self.tick)  # ~33Hz

    def loc_cb(self, m): self.T_mb = pose_to_T(m.pose.pose)
    def mc_cb(self, m): self.T_ob = pose_to_T(m.pose.pose)

    def tick(self):
        if self.T_mb is None or self.T_ob is None:
            return
        T_mo = self.T_mb @ np.linalg.inv(self.T_ob)
        x, y, z, w = mat_to_quat(T_mo[:3, :3])
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame; t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(T_mo[0, 3]); t.transform.translation.y = float(T_mo[1, 3])
        t.transform.translation.z = float(T_mo[2, 3])
        t.transform.rotation.x = x; t.transform.rotation.y = y; t.transform.rotation.z = z; t.transform.rotation.w = w
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(M2O())


if __name__ == '__main__':
    main()
