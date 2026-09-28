# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""YOLO detection, tracking and counting on the Origin One camera.

Subscribes: image (sensor_msgs/Image), camera_info, cloud (the Ouster PointCloud2) and depth
(sensor_msgs/Image, 32FC1 m or 16UC1 mm, aligned with the colour image).
3D object positions come from the lidar points that project into each box (nearest cluster);
the depth image is the fallback. The lidar reaches much further than RealSense depth and, in
Gazebo, also sees the animated people that the simulated depth camera does not render. Runs Ultralytics YOLO with its built-in tracker
(ByteTrack / BoT-SORT) on every frame it can keep up with (older frames are dropped).

Publishes (all relative to the node namespace, /perception by default):
  annotated_image  sensor_msgs/Image           boxes, class, track ID, object ID, counting line
  detections       vision_msgs/Detection2DArray  id = tracker ID, class name + score
  count            std_msgs/Int32              unique objects (3D registry, all counted classes)
  line_count       std_msgs/Int32              unique tracks that crossed the counting line
  counts           std_msgs/String             JSON with per-class numbers
  markers          visualization_msgs/MarkerArray  unique objects in the map frame + labels

Runs in its own process, independent of the lidar / navigation executors.
"""

import json
import os
import time

from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Int32, String
import tf2_ros
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from origin_yolo_perception.counting import (front_surface, LineCrossingCounter,
                                             ObjectRegistry, suppress_overlaps,
                                             TrackClassVoter)


def _quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class YoloPerception(Node):

    def __init__(self):
        super().__init__("yolo_perception")
        p = self.declare_parameter
        self.model_path = p("model", "yolo11s.pt").value
        self.device = p("device", "cuda:0").value
        self.conf = p("confidence", 0.4).value
        self.iou = p("iou", 0.5).value
        self.imgsz = p("image_size", 640).value
        self.tracker_cfg = p("tracker", "bytetrack.yaml").value
        classes = p("classes", [""]).value            # empty: all COCO classes
        self.count_classes = set(c for c in p("count_classes", [""]).value if c)
        self.min_hits = p("min_hits", 5).value         # frames before a track is counted
        self.min_track_score = p("min_track_score", 0.5).value  # mean confidence to count
        self.global_frame = p("global_frame", "map").value
        self.optical_frame = p("optical_frame", "").value  # override if the image frame is not optical
        self.max_depth = p("max_depth", 8.0).value
        self.max_lidar_range = p("max_lidar_range", 20.0).value
        self.fixed_frame = p("fixed_frame", "odom").value  # for lidar/camera time sync
        line_pos = p("line_position", 0.5).value        # fraction of the image width
        # Classes counted at the line. With a moving robot, static objects sweep across the
        # image too, so by default only people (who move themselves) are counted crossing.
        self.line_classes = set(c for c in p("line_classes", ["person"]).value if c)
        assoc_radius = p("association_radius", 0.8).value
        moving = p("moving_classes", ["person"]).value

        import torch  # imported here so a missing CUDA runtime gives a clear error
        from ultralytics import YOLO
        if self.device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warn("CUDA not available, running YOLO on the CPU")
            self.device = "cpu"
        # Bare weight names are looked up in ORIGIN_YOLO_MODELS (default /opt/models, filled by
        # docker/Dockerfile) so nothing is downloaded at run time on the robot.
        if os.sep not in self.model_path:
            local = os.path.join(os.environ.get("ORIGIN_YOLO_MODELS", "/opt/models"),
                                 self.model_path)
            if os.path.exists(local):
                self.model_path = local
        self.model = YOLO(self.model_path)
        self.names = self.model.names
        name_to_id = {v: k for k, v in self.names.items()}
        wanted = [c for c in classes if c]
        self.class_ids = [name_to_id[c] for c in wanted if c in name_to_id] or None
        unknown = [c for c in wanted + list(self.count_classes) if c not in name_to_id]
        if unknown:
            self.get_logger().warn(f"Unknown class names ignored: {unknown}")
        self.precision = 16 if self.device.startswith("cuda") else 32  # FP16 on the GPU

        self.line_ratio = line_pos
        self.line: LineCrossingCounter = None
        self.registry = ObjectRegistry(assoc_radius=assoc_radius, moving_classes=moving)
        self.hits = {}           # track id -> frames seen
        self.score_sum = {}      # track id -> sum of confidences
        self.track_class = {}    # track id -> class name
        self.track_object = {}   # track id -> registry object id
        self.unique_tracks = {}  # class -> set of confirmed track ids
        self.depth = None
        self.cloud = None        # (Nx3 float32 points, frame_id, stamp)
        self.K = None
        self.voter = TrackClassVoter()
        self.busy = False
        self.stats = {"frames": 0, "infer_ms": 0.0}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_img = self.create_publisher(Image, "annotated_image", qos_profile_sensor_data)
        self.pub_det = self.create_publisher(Detection2DArray, "detections", 10)
        self.pub_count = self.create_publisher(Int32, "count", 10)
        self.pub_line = self.create_publisher(Int32, "line_count", 10)
        self.pub_counts = self.create_publisher(String, "counts", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "markers", 2)

        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "depth", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "cloud", self.on_cloud, qos_profile_sensor_data)
        if p("compressed", False).value:  # e.g. the robot's camera over Wi-Fi
            self.create_subscription(CompressedImage, "image", self.on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, "image", self.on_image, qos_profile_sensor_data)
        self.create_timer(5.0, self.log_stats)

        # Warm-up so the first real frame is not slow (CUDA context, kernels).
        self.model.predict(np.zeros((480, 640, 3), np.uint8), device=self.device,
                           quantize=self.precision, verbose=False)
        self.get_logger().info(
            f"YOLO '{self.model_path}' on {self.device} (FP{self.precision}), "
            f"tracker {self.tracker_cfg}, classes: {wanted or 'all'}")

    # ------------------------------------------------------------------ inputs
    def on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def on_depth(self, msg: Image):
        if msg.encoding == "32FC1":
            d = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        elif msg.encoding in ("16UC1", "mono16"):
            d = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width) / 1000.0
        else:
            return
        self.depth = d

    def on_cloud(self, msg: PointCloud2):
        offs = {f.name: f.offset for f in msg.fields if f.datatype == 7}  # FLOAT32
        if not all(k in offs for k in ("x", "y", "z")):
            return
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, np.uint8).reshape(n, msg.point_step)
        pts = np.stack([raw[:, offs[k]:offs[k] + 4].copy().view(np.float32)[:, 0]
                        for k in ("x", "y", "z")], axis=1)
        pts = pts[np.isfinite(pts).all(axis=1)]
        self.cloud = (pts, msg.header.frame_id, Time.from_msg(msg.header.stamp))

    def on_compressed(self, msg: CompressedImage):
        import cv2
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        h, w = bgr.shape[:2]
        self.on_image(Image(header=msg.header, height=h, width=w, encoding="bgr8",
                            step=w * 3, data=bgr.tobytes()))

    def on_image(self, msg: Image):
        if self.busy:
            return
        self.busy = True
        try:
            self.process(msg)
        finally:
            self.busy = False

    # ------------------------------------------------------------------ main step
    def process(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.get_logger().warn(f"Unsupported encoding {msg.encoding}", throttle_duration_sec=10)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        bgr = img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()
        if self.line is None:
            self.line = LineCrossingCounter(self.line_ratio * msg.width)

        t0 = time.perf_counter()
        res = self.model.track(bgr, persist=True, tracker=self.tracker_cfg, conf=self.conf,
                               iou=self.iou, imgsz=self.imgsz, classes=self.class_ids,
                               device=self.device, quantize=self.precision, verbose=False)[0]
        self.stats["infer_ms"] += (time.perf_counter() - t0) * 1000.0
        self.stats["frames"] += 1

        stamp = Time.from_msg(msg.header.stamp)
        cam_frame = self.optical_frame or msg.header.frame_id
        cam_to_map = self.lookup(cam_frame, stamp)
        lidar_uvz = self.project_cloud(cam_frame, stamp, msg.width, msg.height)

        det_array = Detection2DArray(header=msg.header)
        observations = []
        boxes = res.boxes
        if boxes is not None and len(boxes):
            xyxy = boxes.xyxy.cpu().numpy()
            cls = boxes.cls.cpu().numpy().astype(int)
            conf = boxes.conf.cpu().numpy()
            ids = boxes.id.cpu().numpy().astype(int) if boxes.id is not None else [None] * len(cls)
            # 3D position of every box, then one box per physical object (suppress_overlaps).
            positions = [self.position(*b, cam_to_map, lidar_uvz) for b in xyxy]
            keep = set(suppress_overlaps([(*b, sc) for b, sc in zip(xyxy, conf)], positions))
            for k, ((x1, y1, x2, y2), c, s, tid) in enumerate(zip(xyxy, cls, conf, ids)):
                name = self.names[int(c)]
                det = Detection2D(header=msg.header)
                det.bbox.center.position.x = float((x1 + x2) / 2)
                det.bbox.center.position.y = float((y1 + y2) / 2)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = name
                hyp.hypothesis.score = float(s)
                det.results.append(hyp)
                det.id = "" if tid is None else str(tid)
                det_array.detections.append(det)
                if tid is None or k not in keep:
                    continue

                name = self.voter.update(tid, name, float(s))  # stable class per track
                self.track_class[tid] = name
                self.hits[tid] = self.hits.get(tid, 0) + 1
                self.score_sum[tid] = self.score_sum.get(tid, 0.0) + float(s)
                counted = not self.count_classes or name in self.count_classes
                confident = self.score_sum[tid] / self.hits[tid] >= self.min_track_score
                if self.hits[tid] < self.min_hits or not counted or not confident:
                    continue
                self.unique_tracks.setdefault(name, set()).add(tid)
                if not self.line_classes or name in self.line_classes:
                    self.line.update(tid, name, (x1 + x2) / 2)
                p = positions[k]
                if p is not None:
                    observations.append((tid, name, *p))

        if observations:
            self.track_object.update(
                self.registry.update(stamp.nanoseconds * 1e-9, observations))

        self.pub_det.publish(det_array)
        self.publish_counts()
        self.publish_image(res, msg, bgr)
        self.publish_markers(msg.header.stamp)

    # ------------------------------------------------------------------ geometry
    def lookup(self, frame, stamp):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame, frame, stamp, timeout=rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            try:  # slightly stale TF is fine for placing objects
                t = self.tf_buffer.lookup_transform(self.global_frame, frame, Time())
            except Exception:
                return None
        q = t.transform.rotation
        tr = t.transform.translation
        return _quat_to_rot(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z])

    def project_cloud(self, cam_frame, stamp, width, height):
        """Lidar points in the camera optical frame at the image time: (u, v, x, y, z) rows."""
        if self.cloud is None or self.K is None:
            return None
        pts, frame, cloud_stamp = self.cloud
        try:  # camera pose at image time <- lidar pose at scan time, via the odometry frame
            t = self.tf_buffer.lookup_transform_full(
                cam_frame, stamp, frame, cloud_stamp, self.fixed_frame,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform(cam_frame, frame, Time())
            except Exception:
                return None
        q, tr = t.transform.rotation, t.transform.translation
        R = _quat_to_rot(q.x, q.y, q.z, q.w)
        pc = pts @ R.T + np.array([tr.x, tr.y, tr.z])
        pc = pc[(pc[:, 2] > 0.3) & (pc[:, 2] < self.max_lidar_range)]
        fx, fy, px, py = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        u = fx * pc[:, 0] / pc[:, 2] + px
        v = fy * pc[:, 1] / pc[:, 2] + py
        ok = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        return np.column_stack([u[ok], v[ok], pc[ok]])

    def position(self, x1, y1, x2, y2, cam_to_map, lidar_uvz):
        """3D centre of the object in the global frame: nearest lidar cluster inside the box,
        else the median depth of the central box region."""
        if cam_to_map is None:
            return None
        R, t = cam_to_map
        if lidar_uvz is not None and len(lidar_uvz):
            w, h = x2 - x1, y2 - y1
            u, v = lidar_uvz[:, 0], lidar_uvz[:, 1]
            # Central 40 % of the width: the detected object, not occluders at the box edges.
            inside = ((u > x1 + 0.3 * w) & (u < x2 - 0.3 * w) &
                      (v > y1 + 0.1 * h) & (v < y2 - 0.1 * h))
            sel = lidar_uvz[inside, 2:]
            if len(sel) >= 3:
                cluster = sel[front_surface(sel[:, 2])]  # the object, not occluder/background
                return tuple(R @ cluster.mean(axis=0) + t)
        if self.depth is None or self.K is None:
            return None
        h, w = self.depth.shape
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        hw, hh = max(2, (x2 - x1) * 0.15), max(2, (y2 - y1) * 0.15)
        u0, u1 = int(max(0, cx - hw)), int(min(w, cx + hw + 1))
        v0, v1 = int(max(0, cy - hh)), int(min(h, cy + hh + 1))
        patch = self.depth[v0:v1, u0:u1]
        patch = patch[np.isfinite(patch) & (patch > 0.1) & (patch < self.max_depth)]
        if patch.size < 5:
            return None
        z = float(np.median(patch))
        fx, fy, px, py = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        p_cam = np.array([(cx - px) * z / fx, (cy - py) * z / fy, z])  # optical frame
        return tuple(R @ p_cam + t)

    # ------------------------------------------------------------------ outputs
    def publish_counts(self):
        reg = self.registry.counts()
        self.pub_count.publish(Int32(data=self.registry.total))
        self.pub_line.publish(Int32(data=self.line.total if self.line else 0))
        classes = sorted(set(reg) | set(self.unique_tracks))
        self.pub_counts.publish(String(data=json.dumps({
            "unique_objects": reg,
            "unique_objects_total": self.registry.total,
            "unique_tracks": {c: len(self.unique_tracks.get(c, ())) for c in classes},
            "line_crossings": {
                "left_to_right": self.line.left_to_right if self.line else 0,
                "right_to_left": self.line.right_to_left if self.line else 0,
                "per_class": self.line.per_class if self.line else {},
            },
        }, sort_keys=True)))

    def publish_image(self, res, msg, bgr):
        import cv2
        out = res.plot(img=bgr, line_width=2, font_size=14)
        h, w = out.shape[:2]
        lx = int(self.line.line_x)
        cv2.line(out, (lx, 0), (lx, h), (0, 255, 255), 2)
        if res.boxes is not None and res.boxes.id is not None:
            for (x1, _, _, y2), tid in zip(res.boxes.xyxy.cpu().numpy(),
                                           res.boxes.id.cpu().numpy().astype(int)):
                oid = self.track_object.get(int(tid))
                if oid is not None:
                    cv2.putText(out, f"obj #{oid}", (int(x1) + 3, int(y2) - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        # Counts banner on top, large enough to read in a small RViz image panel.
        reg = self.registry.counts()
        line1 = "UNIQUE: " + ("  ".join(f"{k} {v}" for k, v in sorted(reg.items())) or "-")
        line2 = (f"IN VIEW: {0 if res.boxes is None else len(res.boxes)}   "
                 f"LINE ({'/'.join(sorted(self.line_classes)) or 'all'}): "
                 f"L>R {self.line.left_to_right}  R>L {self.line.right_to_left}")
        cv2.rectangle(out, (0, 0), (w, 58), (0, 0, 0), -1)
        cv2.putText(out, line1, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2,
                    cv2.LINE_AA)
        cv2.putText(out, line2, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2,
                    cv2.LINE_AA)
        rgb = out[:, :, ::-1] if msg.encoding == "rgb8" else out
        img = Image(header=msg.header, height=h, width=w, encoding=msg.encoding,
                    step=w * 3, data=np.ascontiguousarray(rgb).tobytes())
        self.pub_img.publish(img)

    def publish_markers(self, stamp):
        arr = MarkerArray()
        palette = {"person": (1.0, 0.3, 0.2), "chair": (0.2, 0.6, 1.0)}
        for obj in self.registry.objects:
            r, g, b = palette.get(obj.cls, (0.9, 0.8, 0.2))
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns = "objects"
            m.id = obj.object_id
            m.type = Marker.CYLINDER if obj.cls == "person" else Marker.CUBE
            m.pose.position = Point(x=obj.x, y=obj.y, z=max(obj.z, 0.2))
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.8
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns = "labels"
            t.id = obj.object_id
            t.type = Marker.TEXT_VIEW_FACING
            t.pose.position = Point(x=obj.x, y=obj.y, z=max(obj.z, 0.2) + 0.5)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = f"{obj.cls} #{obj.object_id}"
            arr.markers.append(t)
        self.pub_markers.publish(arr)

    def log_stats(self):
        f = self.stats["frames"]
        if f:
            self.get_logger().info(
                f"{f / 5.0:.1f} fps, inference {self.stats['infer_ms'] / f:.1f} ms/frame, "
                f"unique objects {self.registry.counts()}, line crossings {self.line.total}")
        self.stats = {"frames": 0, "infer_ms": 0.0}


def main():
    rclpy.init()
    node = YoloPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
