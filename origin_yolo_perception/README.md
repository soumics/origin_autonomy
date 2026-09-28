# origin_yolo_perception

YOLO detection, tracking and counting on the Origin One camera (RealSense D435; Gazebo RGB-D
camera in simulation). It runs in its own process, independent of the lidar, SLAM and
navigation executors.

```bash
ros2 launch origin_yolo_perception perception.launch.py use_sim_time:=true sim:=true
# real robot: sim:=false (RealSense topics /camera/camera/..., Ouster /ouster/points)
# other weights: model:=yolo26s.pt
```

## Pipeline

1. **Detect and track:** Ultralytics YOLO with the built-in ByteTrack
   (`config/bytetrack_origin.yaml`, track buffer 60 frames) runs on the GPU in FP16. Frames
   that arrive while a frame is still being processed are dropped.
2. **Stable class per track:** majority vote over the track's detections. The detector can
   flicker between classes, e.g. "chair" and "airplane" for a black office chair.
3. **One box per object:** boxes that overlap in the image and lie within 0.5 m of each other
   in 3D are duplicates (full body + upper body, "chair" + "bench" on one chair). Identical
   objects side by side overlap in an oblique view too, but lie apart in 3D, so they are kept.
4. **3D position:** Ouster points projected into the box (central 40% of its width); the
   position is the nearest depth cluster with at least 25% of the largest cluster, so thin
   occluders in front (door jambs) and the background behind (walls) are ignored. The depth
   image is the fallback. The lidar reaches much further than RealSense depth, and in Gazebo
   it also sees the animated people, which the simulated depth camera does not render.
5. **Counting**, only for tracks seen for 5 or more frames with a mean confidence of 0.5 or more:
   - **Unique objects** (`count`): a 3D registry in the `map` frame (`counting.ObjectRegistry`).
     - A new track attaches to a known object of the same class only if that object is close
       (0.8 m) and not already claimed by another visible track.
     - Two objects seen in the same frame at least 0.4 m apart are *proven distinct* and never
       merged, however identical they look. Otherwise, same-class objects within 0.8 m are
       merged, which removes duplicates caused by depth noise or by revisiting an object.
     - People may re-associate over walking speed × time (for up to 120 s after last seen).
   - **Line crossings** (`line_count`): each track counts once per direction when its box
     centre crosses the vertical line at `line_position` (with hysteresis).

## Topics (namespace `/perception`)

| Topic | Type | Content |
|---|---|---|
| `annotated_image` | `sensor_msgs/Image` | boxes, class, track ID, object ID, counting line, counts |
| `detections` | `vision_msgs/Detection2DArray` | `id` = track ID, class name + score |
| `count` | `std_msgs/Int32` | unique objects (all counted classes) |
| `line_count` | `std_msgs/Int32` | tracks that crossed the line |
| `counts` | `std_msgs/String` | JSON: per-class unique objects, unique tracks, line crossings |
| `markers` | `visualization_msgs/MarkerArray` | unique objects in `map` with labels |

## Model choice (Ultralytics 8.4.164, confirmed 2026-09-28)

YOLO11s is the default. On frames from the Gazebo world (`origin_office_people`) it gets
every chair right. YOLO26 (the newest family) mislabels these renders:

| View (truth) | yolo11s | yolo11m | yolo26s | yolo26m |
|---|---|---|---|---|
| hall, 3 office chairs | 3 chair | 3 chair | 3 chair | 2 car |
| SE room, 2 office chairs | 2 chair | 2 chair | 2 airplane | – |
| NW room, 2 dining chairs | 2 chair | 2 chair | 2 chair + bench | couch + bed |
| hall, walking person | 1 person | 1 person | 1 person | 1 person |

On real camera images YOLO26 is expected to do at least as well; `model:=yolo26s.pt` switches.
On the RTX 5070 Laptop GPU: 8–10 ms per frame (FP16), so the node keeps up with the camera.

## Results in Gazebo (`origin_office_people`, autonomous exploration with look-around)

Truth: 6 people (3 identical walkers), 7 chairs (5 identical office chairs, 2 identical dining
chairs), 1 couch, 1 refrigerator, 1 dining table.

| Run | person | chair | couch | refrigerator | dining table | other |
|---|---|---|---|---|---|---|
| Nav2 backend | 7 | 10 | 2 | 1 | 1 | bench 2 |
| custom backend | 6 | 6 | 1 | 2 | 1 | bench 1 |

- **Found:** every real object at its true position, including all identical objects. The 3
  identical walkers were counted exactly once each, and the row of 3 identical office chairs
  counted as 3.
- **Missed:** the "Casual female" model was never recognized as a person in the Nav2 run.
- **Errors left:** detector misclassifications of the Gazebo renders: dining chair as
  "bench", office chair as "person", occasional fridge or couch false positives. On the real
  robot, or with a model fine-tuned on these renders, these would be expected to drop.

## Tests

`python3 -m pytest test/` covers line crossing, the registry (identical objects kept apart,
revisits merged, walkers re-associated, two boxes on one body), overlap suppression and
depth-cluster selection.
