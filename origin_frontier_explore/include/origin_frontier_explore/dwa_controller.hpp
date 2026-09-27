// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Dynamic Window Approach local controller for a differential / skid-steer base.
//
// Works in the robot frame (robot at the origin, facing +x). Obstacles are 2D points in that
// frame (e.g. the Ouster cloud, filtered to the robot's height band). For every reachable
// (v, w) in the dynamic window it rolls out a constant-velocity arc, rejects arcs where the
// rectangular footprint (plus margin) hits an obstacle, and scores the rest on progress to a
// target point, heading to the target, clearance and speed.

#pragma once

#include <utility>
#include <vector>

namespace origin_frontier_explore
{

struct DwaParams
{
  double max_v{0.4};         // m/s
  double min_v{0.0};         // m/s (no reversing in normal driving)
  double max_w{1.0};         // rad/s
  double acc_v{1.0};         // m/s^2
  double acc_w{2.5};         // rad/s^2
  double control_period{0.1};  // s, dynamic window = acc * period
  double sim_time{2.0};      // s
  double sim_dt{0.1};        // s
  int v_samples{7};
  int w_samples{21};
  double half_length{0.33};  // m, footprint (x)
  double half_width{0.29};   // m, footprint (y)
  double margin{0.06};       // m, extra clearance around the footprint
  double w_progress{2.0};
  double w_heading{0.8};
  double w_clearance{0.15};
  double w_speed{0.6};
  double local_size{8.0};    // m, side of the local obstacle grid
  double local_resolution{0.05};
};

struct DwaCommand
{
  double v{0.0};
  double w{0.0};
  double clearance{0.0};
  std::vector<std::pair<double, double>> trajectory;  // robot-frame points of the chosen arc
};

class DwaController
{
public:
  explicit DwaController(const DwaParams & params = DwaParams());

  // Obstacle points in the robot frame. Rebuilds the local distance field.
  void setObstacles(const std::vector<std::pair<double, double>> & points);

  // Best command towards (tx, ty) in the robot frame, given the current velocity.
  // Returns false if every arc in the dynamic window collides.
  bool compute(double cur_v, double cur_w, double tx, double ty, DwaCommand & out) const;

  // Footprint clearance (m) at a robot-frame pose; <= 0 means inside obstacle + margin.
  double clearance(double x, double y, double th) const;

  const DwaParams & params() const {return params_;}

private:
  double distanceAt(double x, double y) const;
  // Distance to the nearest obstacle for every footprint sample at a robot-frame pose.
  std::vector<double> sampleDistances(double x, double y, double th) const;

  DwaParams params_;
  int cells_{0};
  std::vector<float> dist_;
  std::vector<std::pair<double, double>> footprint_samples_;
};

}  // namespace origin_frontier_explore
