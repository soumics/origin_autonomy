#!/usr/bin/env python3
# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Check that the Origin One's ROS 2 interface is available on this computer.

Compares what is visible (normally through the Zenoh bridge) with the interface documented by
Avular (origin_one 1.0-3: software_development/ros2/ros2_interfaces): each topic's presence, type
and publisher/subscriber, and the measured rate of the published topics. Also lists every other
/robot topic and service found.

    check_origin_one.py [--discovery 6] [--rate-window 4]    # exit code 0 if all essential OK
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

# (name, type, direction, essential, what). direction: "pub" = the robot publishes it,
# "sub" = the robot subscribes to it (our commands go there).
EXPECTED = [
    ("/robot/lidar/points", "sensor_msgs/msg/PointCloud2", "pub", True, "Ouster lidar"),
    ("/robot/odom", "nav_msgs/msg/Odometry", "pub", True, "wheel odometry"),
    ("/robot/tf", "tf2_msgs/msg/TFMessage", "pub", False,
     "TF odom->base_link (else derived from /robot/odom)"),
    ("/robot/camera/color/image_raw", "sensor_msgs/msg/Image", "pub", False, "camera colour"),
    ("/robot/camera/color/camera_info", "sensor_msgs/msg/CameraInfo", "pub", False,
     "camera colour info"),
    ("/robot/camera/depth/image_raw", "sensor_msgs/msg/Image", "pub", False, "camera depth"),
    ("/robot/camera/depth/camera_info", "sensor_msgs/msg/CameraInfo", "pub", False,
     "camera depth info"),
    ("/robot/cmd_vel", "geometry_msgs/msg/Twist", "pub", False, "velocity reference"),
    ("/robot/cmd_vel_controller/control_mode", "origin_msgs/msg/ControlMode", "pub", False,
     "control mode (docs)"),
    ("/robot/control_mode", "origin_msgs/msg/ControlMode", "pub", False,
     "control mode (seen on a real robot)"),
    ("/robot/cmd_vel_user", "geometry_msgs/msg/Twist", "sub", True, "velocity input (USER)"),
]
SERVICES = [
    ("/robot/cmd_vel_controller/set_control_mode", "origin_msgs/srv/SetControlMode", True),
    ("/robot/cmd_vel_controller/reset_control_mode", None, False),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--discovery", type=float, default=6.0)
    ap.add_argument("--rate-window", type=float, default=4.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = Node("origin_one_check")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.discovery:
        rclpy.spin_once(node, timeout_sec=0.2)
    topics = dict(node.get_topic_names_and_types())
    services = dict(node.get_service_names_and_types())

    # Measure rates of the published topics that exist.
    counts, subs = {}, []
    for name, typ, direction, _, _ in EXPECTED:
        if direction == "pub" and name in topics:
            try:
                msg_type = get_message(topics[name][0])
            except Exception:  # noqa: BLE001 - type not installed here
                continue
            counts[name] = 0
            subs.append(node.create_subscription(
                msg_type, name, lambda _m, n=name: counts.__setitem__(n, counts[n] + 1),
                qos_profile_sensor_data))
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.rate_window:
        rclpy.spin_once(node, timeout_sec=0.05)

    # Lidar payload: what the Wi-Fi link has to carry, raw vs. the robot's filtered cloud.
    sizes = {}
    from sensor_msgs.msg import PointCloud2
    for name in ("/robot/lidar/points", "/robot/lidar/points_filtered"):
        if name in topics:
            got = []
            s = node.create_subscription(PointCloud2, name, got.append, qos_profile_sensor_data)
            t0 = time.monotonic()
            while len(got) < 3 and time.monotonic() - t0 < 6.0:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_subscription(s)
            if got:
                m = got[-1]
                n = m.width * m.height
                span = time.monotonic() - t0
                sizes[name] = (len(m.data), n, m.point_step, m.header.frame_id,
                               [f.name for f in m.fields], len(got) / max(span, 1e-3))

    # Self-hits: filtered lidar points inside the robot's turning circle (0.55 m) block the
    # collision monitor. Needs the running stack (our self-filtered cloud + TF).
    selfhits = None
    if "/origin/lidar/points_filtered" in topics:
        import numpy as np
        from tf2_ros import Buffer, TransformListener
        buf = Buffer()
        TransformListener(buf, node)
        got = []
        s = node.create_subscription(PointCloud2, "/origin/lidar/points_filtered", got.append,
                                     qos_profile_sensor_data)
        t0 = time.monotonic()
        while not got and time.monotonic() - t0 < 8.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            m = got[-1]
            try:
                from rclpy.time import Time
                tr = buf.lookup_transform("base_link", m.header.frame_id, Time()).transform
                raw = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
                off = {f.name: f.offset for f in m.fields}
                xyz = np.stack([raw[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0]
                                for k in "xyz"], 1).astype(float)
                xyz = xyz[np.isfinite(xyz).all(1)]
                q = tr.rotation
                x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
                b = xyz @ R.T + [tr.translation.x, tr.translation.y, tr.translation.z]
                band = (b[:, 2] > 0.08) & (b[:, 2] < 0.8)
                r = np.hypot(b[:, 0], b[:, 1])
                close = b[band & (r < 0.55)]
                selfhits = (len(close), close[:8].round(2).tolist())
            except Exception as e:  # noqa: BLE001
                selfhits = (-1, str(e))

    ok_all = True
    rows = []
    for name, typ, direction, essential, what in EXPECTED:
        if name not in topics:
            status, detail = "MISSING", ""
        elif typ not in topics[name]:
            status, detail = "WRONG TYPE", ",".join(topics[name])
        elif direction == "pub":
            n_pub = node.count_publishers(name)
            hz = counts.get(name, 0) / args.rate_window
            status = "OK" if n_pub and hz > 0 else ("NO DATA" if n_pub else "NO PUBLISHER")
            detail = f"{hz:5.1f} Hz"
        else:
            status = "OK" if node.count_subscribers(name) else "NO SUBSCRIBER"
            detail = "robot listens"
        if status != "OK" and essential:
            ok_all = False
        rows.append((status, name, what + ("" if essential else " (optional)"), detail))
    for name, typ, essential in SERVICES:
        if name not in services:
            status = "MISSING"
        elif typ and typ not in services[name]:
            status = "WRONG TYPE"
        else:
            status = "OK"
        if status != "OK" and essential:
            ok_all = False
        rows.append((status, name, "service" + ("" if essential else " (optional)"), ""))

    bar = "=" * 94
    print(bar)
    print("  Origin One ROS 2 interface on this computer (Avular documentation 1.0-3)")
    print(bar)
    for status, name, what, detail in rows:
        mark = "OK " if status == "OK" else "!! "
        print(f"  {mark}{status:13s} {name:46s} {detail:>13s}  {what}")
    known = {e[0] for e in EXPECTED} | {s[0] for s in SERVICES}
    extra_t = sorted(t for t in topics if t.startswith("/robot/") and t not in known)
    extra_s = sorted(s for s in services if s.startswith("/robot/") and s not in known
                     and not s.split("/")[-1] in ("describe_parameters", "get_parameter_types",
                                                   "get_parameters", "list_parameters",
                                                   "set_parameters", "get_type_description",
                                                   "set_parameters_atomically"))
    print(f"  Other /robot topics ({len(extra_t)}): " + (", ".join(extra_t) or "-"))
    print(f"  Other /robot services ({len(extra_s)}): " + (", ".join(extra_s) or "-"))
    for name, (nbytes, n, step, frame, fields, hz) in sizes.items():
        print(f"  {name}: {nbytes / 1e6:.2f} MB/msg, {n} points x {step} B, frame {frame}, "
              f"~{hz:.1f} Hz received -> {nbytes * 8 * hz / 1e6:.0f} Mbit/s; fields {fields}")
    if selfhits is not None:
        print(f"  Lidar points inside the robot's turning circle (0.55 m, 0.08-0.8 m high): "
              f"{selfhits[0]}  e.g. {selfhits[1]}")
    tf_topics = sorted(t for t, ty in topics.items() if "tf2_msgs/msg/TFMessage" in ty)
    print("  TF topics: " + (", ".join(tf_topics) or "none"))
    for v in ("/robot/cmd_vel_user", "/robot/cmd_vel", "/robot/cmd_vel_joy", "/robot/cmd_vel_aut"):
        if v in topics:
            subs = [i.node_name for i in node.get_subscriptions_info_by_topic(v)]
            pubs = [i.node_name for i in node.get_publishers_info_by_topic(v)]
            print(f"  {v}: subscribers {subs or '-'}, publishers {pubs or '-'}")
    print(f"  RESULT: {'all essential interfaces available' if ok_all else 'ESSENTIAL INTERFACES MISSING'}")
    print(bar, flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
