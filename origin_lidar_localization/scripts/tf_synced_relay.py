#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Republish a sensor topic once its TF to `fixed_frame` is available (visualization helper).

RViz (Jazzy) drops a message whose transform to the fixed frame is not available when the
message arrives. With RTAB-Map, map -> icp_odom -> odom is only complete after icp_odometry has
processed the scan (~40 ms later), so the raw Ouster cloud shows "Could not transform from
[os_lidar] to [map]". This relay holds each cloud until map -> cloud frame exists at the cloud's
stamp and then republishes it unchanged on `output`. Only RViz uses the output; the robot's
consumers keep the raw topic without added latency.
"""

from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


class TfSyncedRelay(Node):

    def __init__(self):
        super().__init__("tf_synced_relay")
        self.fixed_frame = self.declare_parameter("fixed_frame", "map").value
        self.max_wait = self.declare_parameter("max_wait", 1.0).value  # s (message time)
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        self.pending = deque(maxlen=20)
        self.pub = self.create_publisher(PointCloud2, "output", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "input", lambda m: self.pending.append(m),
                                 qos_profile_sensor_data)
        self.create_timer(0.01, self.flush)

    def flush(self):
        while self.pending:
            msg = self.pending[0]
            stamp = Time.from_msg(msg.header.stamp)
            if self.buffer.can_transform(self.fixed_frame, msg.header.frame_id, stamp):
                self.pub.publish(self.pending.popleft())
                continue
            newest = Time.from_msg(self.pending[-1].header.stamp)
            if (newest - stamp) > Duration(seconds=self.max_wait):
                self.pending.popleft()  # TF never arrived for this one; drop it
                continue
            break


def main():
    rclpy.init()
    rclpy.spin(TfSyncedRelay())


if __name__ == "__main__":
    main()
