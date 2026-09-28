# Copyright 2026 Soumic Sarkar
# SPDX-License-Identifier: Apache-2.0

import numpy as np

from origin_yolo_perception.counting import (front_surface, LineCrossingCounter,
                                             ObjectRegistry, suppress_overlaps,
                                             TrackClassVoter)


def test_line_crossing_counts_each_track_once_per_direction():
    c = LineCrossingCounter(line_x=320, hysteresis=5)
    for x in (100, 200, 330, 400):      # track 1 left -> right
        c.update(1, "person", x)
    for x in (400, 330, 300, 200):      # back right -> left
        c.update(1, "person", x)
    for x in (100, 400, 100, 400):      # jitter back and forth: no double counting
        c.update(1, "person", x)
    assert c.left_to_right == 1
    assert c.right_to_left == 1


def test_line_crossing_ignores_dead_band_and_distinguishes_tracks():
    c = LineCrossingCounter(line_x=320, hysteresis=10)
    for x in (300, 318, 322, 312):      # never leaves the dead band on the right side
        c.update(1, "chair", x)
    assert c.total == 0
    for tid in (2, 3, 4):               # three identical objects, three separate crossings
        c.update(tid, "chair", 100)
        c.update(tid, "chair", 500)
    assert c.left_to_right == 3
    assert c.per_class == {"chair": 3}


def test_identical_objects_seen_together_stay_separate():
    r = ObjectRegistry(assoc_radius=0.8)
    # Three identical chairs 0.5 m apart (closer than the association radius), same frame.
    r.update(0.0, [(1, "chair", 0.0, 0.0, 0.5), (2, "chair", 0.5, 0.0, 0.5),
                   (3, "chair", 1.0, 0.0, 0.5)])
    assert r.counts() == {"chair": 3}


def test_same_static_object_with_new_track_id_is_not_counted_twice():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "chair", 2.0, 1.0, 0.5), (2, "chair", 4.0, 1.0, 0.5)])
    # Robot turns away and back: the tracker assigns new IDs to the same two chairs.
    ids = r.update(20.0, [(7, "chair", 2.1, 1.05, 0.5), (8, "chair", 3.9, 0.95, 0.5)])
    assert r.counts() == {"chair": 2}
    assert ids[7] != ids[8]


def test_new_track_cannot_steal_object_claimed_by_visible_track():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "chair", 0.0, 0.0, 0.5)])
    # Track 1 still visible, and a new identical chair appears 0.4 m away.
    r.update(1.0, [(1, "chair", 0.0, 0.0, 0.5), (2, "chair", 0.4, 0.0, 0.5)])
    assert r.counts() == {"chair": 2}


def test_moving_person_reassociates_within_speed_gate_but_not_after_memory():
    r = ObjectRegistry(assoc_radius=0.8, moving_speed=1.2, moving_memory=120.0)
    r.update(0.0, [(1, "person", 0.0, 0.0, 1.0)])
    # Occluded for 3 s, reappears 3 m away with a new track ID: same person (gate 0.8 + 3.6 m).
    r.update(3.0, [(2, "person", 3.0, 0.0, 1.0)])
    assert r.counts() == {"person": 1}
    # A person far away (10 m) shortly after: a different person.
    r.update(4.0, [(3, "person", 13.0, 0.0, 1.0)])
    assert r.counts() == {"person": 2}
    # Much later, far from everyone known (beyond the memory window): new object.
    r.update(400.0, [(4, "person", 30.0, 0.0, 1.0)])
    assert r.counts() == {"person": 3}


def test_classes_do_not_mix():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "chair", 0.0, 0.0, 0.5)])
    r.update(5.0, [(2, "couch", 0.1, 0.0, 0.5)])
    assert r.counts() == {"chair": 1, "couch": 1}


def test_noisy_duplicate_of_static_object_is_merged():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "chair", 0.0, 0.0, 0.5)])
    # Later a new track on the same chair, but the depth estimate is 1.0 m off (outside the
    # gate), so a second object is created ...
    r.update(10.0, [(2, "chair", 1.0, 0.0, 0.5)])
    assert r.counts() == {"chair": 2}
    # ... and further observations pull it towards the true position: now within the radius
    # and never seen together with object 1 -> merged into one chair.
    for k in range(5):
        r.update(11.0 + k, [(2, "chair", 0.3, 0.0, 0.5)])
    assert r.counts() == {"chair": 1}


def test_identical_objects_seen_together_are_never_merged():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "chair", 0.0, 0.0, 0.5), (2, "chair", 0.6, 0.0, 0.5)])
    for k in range(10):  # both drift towards each other in later, separate views
        r.update(1.0 + k, [(1, "chair", 0.1, 0.0, 0.5)])
        r.update(1.5 + k, [(2, "chair", 0.4, 0.0, 0.5)])
    assert r.counts() == {"chair": 2}


def test_track_class_vote_is_stable():
    v = TrackClassVoter()
    for cls in ("chair", "airplane", "chair", "chair", "airplane"):
        out = v.update(5, cls)
    assert out == "chair"


def test_suppress_overlaps_keeps_one_box_per_object_but_all_separate_objects():
    boxes = [
        (100, 100, 200, 400, 0.9),   # person, full body
        (105, 100, 195, 250, 0.6),   # same person, upper body (contained)
        (300, 200, 360, 300, 0.8),   # chair A
        (370, 200, 430, 300, 0.8),   # identical chair B right next to it (touching, no overlap)
        (302, 205, 358, 298, 0.5),   # 'bench' box on chair A (IoU > 0.5)
    ]
    assert suppress_overlaps(boxes) == [0, 2, 3]  # 'bench' box lies >90 % inside chair A
    pos = [(5, 0, 1), (5.1, 0, 1.2), (3, 1, 0.5), (3, 1.9, 0.5), (3.05, 1.0, 0.5)]
    assert suppress_overlaps(boxes, pos) == [0, 2, 3]   # with 3D: duplicates removed


def test_identical_chairs_overlapping_in_oblique_view_are_kept():
    # Row of three chairs seen at an angle: boxes overlap by more than 50 % ...
    boxes = [(100, 200, 220, 330, 0.9), (150, 200, 260, 320, 0.8), (190, 205, 290, 315, 0.7)]
    # ... but the lidar puts them 1 m apart.
    pos = [(3.0, -1.0, 0.5), (3.6, -0.2, 0.5), (4.2, 0.6, 0.5)]
    assert suppress_overlaps(boxes, pos) == [0, 1, 2]


def test_front_surface_ignores_thin_occluder_and_background():
    person = np.full(40, 4.0) + np.linspace(-0.1, 0.1, 40)
    jamb = np.full(6, 2.0)            # few points of a door jamb in front
    wall = np.full(15, 7.0)           # background wall behind
    d = np.concatenate([jamb, person, wall])
    sel = front_surface(d)
    assert abs(d[sel].mean() - 4.0) < 0.15


def test_standing_person_revisited_much_later_is_one_person():
    r = ObjectRegistry(assoc_radius=0.8, moving_memory=120.0)
    r.update(0.0, [(1, "person", 9.2, -0.8, 1.0)])
    r.update(500.0, [(2, "person", 9.3, -0.7, 1.0)])   # long after the memory window
    assert r.counts() == {"person": 1}


def test_two_identical_people_standing_side_by_side_stay_two():
    r = ObjectRegistry(assoc_radius=0.8)
    r.update(0.0, [(1, "person", 0.0, 0.0, 1.0), (2, "person", 0.5, 0.0, 1.0)])
    r.update(300.0, [(3, "person", 0.1, 0.0, 1.0)])
    r.update(301.0, [(4, "person", 0.45, 0.0, 1.0)])
    assert r.counts() == {"person": 2}


def test_two_boxes_on_one_body_in_same_frame_are_merged():
    r = ObjectRegistry(assoc_radius=0.8)
    # Upper-body and lower-body boxes of the same standing person, same frame, 0.15 m apart.
    r.update(0.0, [(1, "person", 9.07, -4.25, 1.2), (2, "person", 9.08, -4.10, 0.5)])
    assert r.counts() == {"person": 1}
