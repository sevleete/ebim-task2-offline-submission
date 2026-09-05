#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState

topic = sys.argv[1]
kind = sys.argv[2] if len(sys.argv) > 2 else "joint"
timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
need = int(sys.argv[4]) if len(sys.argv) > 4 else 3

rclpy.init()
node = Node("wait_topic_probe")
cnt = [0]
msgtype = CompressedImage if kind == "image" else JointState
node.create_subscription(msgtype, topic,
                         lambda m: cnt.__setitem__(0, cnt[0] + 1),
                         qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < timeout and cnt[0] < need:
    rclpy.spin_once(node, timeout_sec=0.2)
alive = cnt[0] >= need
print(f"{'✓' if alive else '✗'} {topic} 收到 {cnt[0]} 条"
      f"(用时 {time.time()-t0:.1f}s)")
rclpy.shutdown()
sys.exit(0 if alive else 1)
