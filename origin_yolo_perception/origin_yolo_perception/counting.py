# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0
"""Counting logic, independent of ROS so it can be unit-tested.

Two complementary counts:

* LineCrossingCounter: image-space. A tracked object is counted once per direction when its
  box centre crosses a vertical line in the image (with hysteresis against jitter).

* ObjectRegistry: world-space unique objects. Every confirmed track is placed in the map frame
  (from depth + TF). A new track is attached to an existing object of the same class only if
  that object is close enough AND not currently claimed by another visible track. Identical-
  looking objects that are seen at the same time therefore stay separate objects, while the
  same object seen again later (new tracker ID after the robot turned away) is not counted
  twice. Two static objects of the same class that end up within the radius (depth noise) are
  merged unless they were ever visible in the same frame: seeing them together proves they
  are different objects. Moving classes (people) may also re-associate over a larger radius
  that grows with the time since the object was last seen (walking speed x time).
"""

from dataclasses import dataclass, field
import math
from typing import Dict, List, Optional, Set, Tuple


class LineCrossingCounter:
    """Counts unique track IDs whose box centre crosses x = line_x (pixels)."""

    def __init__(self, line_x: float, hysteresis: float = 5.0):
        self.line_x = line_x
        self.hysteresis = hysteresis
        self._side: Dict[int, int] = {}          # track id -> -1 (left) / +1 (right)
        self._counted: Dict[int, Set[int]] = {}  # track id -> directions already counted
        self.left_to_right = 0
        self.right_to_left = 0
        self.per_class: Dict[str, int] = {}

    def update(self, track_id: int, cls: str, cx: float) -> Optional[int]:
        """Feed one observation. Returns +1 / -1 if this observation completed a crossing."""
        if cx > self.line_x + self.hysteresis:
            side = 1
        elif cx < self.line_x - self.hysteresis:
            side = -1
        else:
            return None  # inside the dead band: keep the previous side
        prev = self._side.get(track_id)
        self._side[track_id] = side
        if prev is None or prev == side:
            return None
        done = self._counted.setdefault(track_id, set())
        if side in done:
            return None  # this track already crossed in this direction
        done.add(side)
        if side == 1:
            self.left_to_right += 1
        else:
            self.right_to_left += 1
        self.per_class[cls] = self.per_class.get(cls, 0) + 1
        return side

    @property
    def total(self) -> int:
        return self.left_to_right + self.right_to_left


def suppress_overlaps(boxes, positions=None, iou_thresh=0.5, containment_thresh=0.7,
                      same_object_dist=0.5, image_only_iou=0.7, image_only_containment=0.9):
    """Class-agnostic duplicate suppression. boxes: list of (x1, y1, x2, y2, score);
    positions: optional list of 3D points (or None) per box.

    Detectors sometimes put two boxes on one object (full body + upper body, or 'chair' and
    'bench' on the same dining chair). Two such boxes in one frame must not count as two
    objects seen together. But identical objects side by side also produce overlapping boxes
    when seen at an angle, so overlap alone is not enough: with 3D positions, boxes are only
    duplicates if they are also within same_object_dist in 3D. Without positions, stricter
    image-only thresholds apply. Returns the indices to keep (highest score first wins).
    """
    order = sorted(range(len(boxes)), key=lambda i: -boxes[i][4])
    keep = []
    for i in order:
        x1, y1, x2, y2, _ = boxes[i]
        ai = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        dup = False
        for j in keep:
            u1, v1, u2, v2, _ = boxes[j]
            pi = positions[i] if positions else None
            pj = positions[j] if positions else None
            aj = max(0.0, u2 - u1) * max(0.0, v2 - v1)
            iw = max(0.0, min(x2, u2) - max(x1, u1))
            ih = max(0.0, min(y2, v2) - max(y1, v1))
            inter = iw * ih
            if inter <= 0.0:
                continue
            iou = inter / (ai + aj - inter)
            contained = inter / max(1e-9, min(ai, aj))
            if pi is not None and pj is not None:
                d3 = math.dist(pi, pj)
                if d3 > same_object_dist:
                    continue  # different places in 3D: different objects
                if iou >= iou_thresh or contained >= containment_thresh:
                    dup = True
                    break
            elif iou >= image_only_iou or contained >= image_only_containment:
                dup = True
                break
        if not dup:
            keep.append(i)
    return sorted(keep)


def front_surface(depths, bin_size=0.3, min_fraction=0.25):
    """Index mask of the object's points among the depths inside a box: the nearest depth bin
    holding at least min_fraction of the largest bin, +- one bin. A thin occluder in front
    (door jamb) loses against the object; the background behind a chair's open back loses
    against the chair, which is nearer. Callers pass only the points in the central part of
    the box, where the detected object is and occluders at the box edges are not."""
    import numpy as np
    d = np.asarray(depths, dtype=float)
    if d.size == 0:
        return np.zeros(0, dtype=bool)
    lo = d.min()
    bins = np.floor((d - lo) / bin_size).astype(int)
    counts = np.bincount(bins)
    chosen = int(np.nonzero(counts >= min_fraction * counts.max())[0][0])
    return np.abs(bins - chosen) <= 1 if counts[chosen] < 3 else bins == chosen


class TrackClassVoter:
    """Stable class per track: the detector's class can flicker between frames (e.g. an office
    chair seen as 'chair' in one frame and 'airplane' in the next). Majority vote per track."""

    def __init__(self):
        self._votes: Dict[int, Dict[str, float]] = {}

    def update(self, track_id: int, cls: str, score: float = 1.0) -> str:
        v = self._votes.setdefault(track_id, {})
        v[cls] = v.get(cls, 0.0) + score
        return max(v.items(), key=lambda kv: kv[1])[0]


@dataclass
class TrackedObject:
    object_id: int
    cls: str
    x: float
    y: float
    z: float
    last_seen: float
    observations: int = 1
    track_ids: Set[int] = field(default_factory=set)


class ObjectRegistry:
    """Unique objects in a fixed (map) frame, fed by tracker IDs + 3D positions."""

    def __init__(self, assoc_radius: float = 0.8, moving_classes=("person",),
                 moving_speed: float = 1.2, moving_max_radius: float = 6.0,
                 moving_memory: float = 120.0, smoothing: float = 0.3,
                 distinct_separation: float = 0.4):
        self.assoc_radius = assoc_radius
        self.moving_classes = set(moving_classes)
        self.moving_speed = moving_speed
        self.moving_max_radius = moving_max_radius
        self.moving_memory = moving_memory
        self.smoothing = smoothing
        self.distinct_separation = distinct_separation
        self.objects: List[TrackedObject] = []
        self._track_to_object: Dict[int, TrackedObject] = {}
        self._next_id = 1
        # Pairs of object ids that were visible in the same frame: proven to be different
        # objects, however similar they look and however close they are.
        self._seen_together: Set[Tuple[int, int]] = set()

    def update(self, stamp: float,
               observations: List[Tuple[int, str, float, float, float]]) -> Dict[int, int]:
        """observations: (track_id, class, x, y, z) of all tracks visible in one frame.

        Returns {track_id: object_id}.
        """
        visible_tracks = {tid for tid, *_ in observations}
        # Objects claimed by a track that is visible in this frame cannot take a new track.
        claimed = {id(self._track_to_object[t]) for t in visible_tracks
                   if t in self._track_to_object}
        result: Dict[int, int] = {}

        # Known tracks first, so their objects are claimed before new tracks are matched.
        pending = []
        for tid, cls, x, y, z in observations:
            obj = self._track_to_object.get(tid)
            if obj is not None and obj.cls == cls:
                self._move(obj, x, y, z, stamp)
                result[tid] = obj.object_id
            else:
                pending.append((tid, cls, x, y, z))

        # New tracks: nearest unclaimed object of the same class within the gate, else new.
        for tid, cls, x, y, z in pending:
            best, best_d = None, math.inf
            for obj in self.objects:
                if obj.cls != cls or id(obj) in claimed:
                    continue
                d = math.hypot(obj.x - x, obj.y - y)
                if d < self._gate(obj, stamp) and d < best_d:
                    best, best_d = obj, d
            if best is None:
                best = TrackedObject(self._next_id, cls, x, y, z, stamp)
                self._next_id += 1
                self.objects.append(best)
            else:
                self._move(best, x, y, z, stamp)
            best.track_ids.add(tid)
            self._track_to_object[tid] = best
            claimed.add(id(best))
            result[tid] = best.object_id

        # Seen together AND in clearly different places in this frame: proven distinct. Two
        # boxes on one body (upper/lower) are seen together too, but at the same place.
        pos = {}
        for tid, cls, x, y, z in observations:
            pos.setdefault(result[tid], (x, y))
        ids = sorted(pos)
        for i, a in enumerate(ids):
            for b in ids[i + 1:]:
                if math.dist(pos[a], pos[b]) >= self.distinct_separation:
                    self._seen_together.add((a, b))
        self._merge_duplicates()
        return {t: self._track_to_object[t].object_id for t in result}

    def _merge_duplicates(self):
        """Merge objects of the same class that are within the association radius and were
        never visible together: position noise, or a standing person revisited after the
        movement memory expired, split one object into two."""
        merged = True
        while merged:
            merged = False
            for i, a in enumerate(self.objects):
                for b in self.objects[i + 1:]:
                    if b.cls != a.cls:
                        continue
                    key = (min(a.object_id, b.object_id), max(a.object_id, b.object_id))
                    if key in self._seen_together:
                        continue
                    if math.hypot(a.x - b.x, a.y - b.y) >= self.assoc_radius:
                        continue
                    keep, drop = (a, b) if a.observations >= b.observations else (b, a)
                    n = keep.observations + drop.observations
                    keep.x = (keep.x * keep.observations + drop.x * drop.observations) / n
                    keep.y = (keep.y * keep.observations + drop.y * drop.observations) / n
                    keep.z = (keep.z * keep.observations + drop.z * drop.observations) / n
                    keep.observations = n
                    keep.last_seen = max(keep.last_seen, drop.last_seen)
                    keep.track_ids |= drop.track_ids
                    for t in drop.track_ids:
                        self._track_to_object[t] = keep
                    self._seen_together = {
                        (min(keep.object_id if x == drop.object_id else x,
                             keep.object_id if y == drop.object_id else y),
                         max(keep.object_id if x == drop.object_id else x,
                             keep.object_id if y == drop.object_id else y))
                        for x, y in self._seen_together}
                    self.objects.remove(drop)
                    merged = True
                    break
                if merged:
                    break

    def _gate(self, obj: TrackedObject, stamp: float) -> float:
        if obj.cls not in self.moving_classes:
            return self.assoc_radius
        dt = max(0.0, stamp - obj.last_seen)
        if dt > self.moving_memory:
            return 0.0  # too long ago to be sure it is the same person
        return min(self.assoc_radius + self.moving_speed * dt, self.moving_max_radius)

    def _move(self, obj: TrackedObject, x: float, y: float, z: float, stamp: float):
        a = 1.0 if obj.cls in self.moving_classes else self.smoothing
        obj.x += a * (x - obj.x)
        obj.y += a * (y - obj.y)
        obj.z += a * (z - obj.z)
        obj.last_seen = stamp
        obj.observations += 1

    def counts(self) -> Dict[str, int]:
        c: Dict[str, int] = {}
        for obj in self.objects:
            c[obj.cls] = c.get(obj.cls, 0) + 1
        return c

    @property
    def total(self) -> int:
        return len(self.objects)
