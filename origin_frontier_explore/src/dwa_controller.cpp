// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0

#include "origin_frontier_explore/dwa_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "origin_frontier_explore/grid.hpp"

namespace origin_frontier_explore
{

namespace
{
double normalizeAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

DwaController::DwaController(const DwaParams & params)
: params_(params)
{
  cells_ = static_cast<int>(std::ceil(params_.local_size / params_.local_resolution));
  dist_.assign(static_cast<size_t>(cells_) * cells_, 1.0f);

  // Footprint perimeter samples (robot frame), spaced below the grid resolution.
  const double step = params_.local_resolution * 0.8;
  const double L = params_.half_length;
  const double W = params_.half_width;
  for (double x = -L; x <= L + 1e-9; x += step) {
    footprint_samples_.emplace_back(x, W);
    footprint_samples_.emplace_back(x, -W);
  }
  for (double y = -W + step; y < W - 1e-9; y += step) {
    footprint_samples_.emplace_back(L, y);
    footprint_samples_.emplace_back(-L, y);
  }
}

void DwaController::setObstacles(const std::vector<std::pair<double, double>> & points)
{
  const double half = params_.local_size / 2.0;
  std::vector<Cell> cells;
  cells.reserve(points.size());
  for (const auto & [x, y] : points) {
    const int cx = static_cast<int>(std::floor((x + half) / params_.local_resolution));
    const int cy = static_cast<int>(std::floor((y + half) / params_.local_resolution));
    if (cx >= 0 && cy >= 0 && cx < cells_ && cy < cells_) {
      cells.emplace_back(cx, cy);
    }
  }
  dist_ = obstacleDistance(cells_, cells_, params_.local_resolution, cells, 1.0f);
}

double DwaController::distanceAt(double x, double y) const
{
  const double half = params_.local_size / 2.0;
  const int cx = static_cast<int>(std::floor((x + half) / params_.local_resolution));
  const int cy = static_cast<int>(std::floor((y + half) / params_.local_resolution));
  if (cx < 0 || cy < 0 || cx >= cells_ || cy >= cells_) {
    return 1.0;  // outside the local window: treated as free
  }
  return dist_[cy * cells_ + cx];
}

std::vector<double> DwaController::sampleDistances(double x, double y, double th) const
{
  const double c = std::cos(th), s = std::sin(th);
  std::vector<double> d;
  d.reserve(footprint_samples_.size());
  for (const auto & [px, py] : footprint_samples_) {
    d.push_back(distanceAt(x + c * px - s * py, y + s * px + c * py));
  }
  return d;
}

double DwaController::clearance(double x, double y, double th) const
{
  const auto d = sampleDistances(x, y, th);
  return *std::min_element(d.begin(), d.end()) - params_.margin;
}

bool DwaController::compute(
  double cur_v, double cur_w, double tx, double ty, DwaCommand & out) const
{
  const auto & p = params_;
  const double v_lo = std::max(p.min_v, cur_v - p.acc_v * p.control_period);
  const double v_hi = std::min(p.max_v, cur_v + p.acc_v * p.control_period);
  const double w_lo = std::max(-p.max_w, cur_w - p.acc_w * p.control_period);
  const double w_hi = std::min(p.max_w, cur_w + p.acc_w * p.control_period);

  std::vector<double> vs, ws;
  for (int i = 0; i < p.v_samples; ++i) {
    vs.push_back(p.v_samples == 1 ? v_lo : v_lo + (v_hi - v_lo) * i / (p.v_samples - 1));
  }
  if (v_lo > 0.0) {
    vs.push_back(v_lo);  // decelerate as fast as allowed
  } else {
    vs.push_back(0.0);   // rotate in place
  }
  for (int i = 0; i < p.w_samples; ++i) {
    ws.push_back(p.w_samples == 1 ? w_lo : w_lo + (w_hi - w_lo) * i / (p.w_samples - 1));
  }
  ws.push_back(0.0);

  // Per-sample start distances: a footprint point that already starts within the margin may
  // stay there or move away, but not get closer (lets the robot escape after being pushed
  // close to an obstacle, without allowing it to drive further in).
  const std::vector<double> start_d = sampleDistances(0.0, 0.0, 0.0);
  const bool start_inside_margin =
    *std::min_element(start_d.begin(), start_d.end()) < p.margin;

  const int steps = std::max(1, static_cast<int>(std::round(p.sim_time / p.sim_dt)));
  double best_cost = std::numeric_limits<double>::infinity();
  bool found = false;
  for (const double v : vs) {
    for (const double w : ws) {
      if (w < w_lo - 1e-9 || w > w_hi + 1e-9) {
        continue;
      }
      // Roll out the arc until the end of the horizon or the first collision. The arc is
      // admissible if the robot can still brake to a stop before that collision point
      // (classic DWA: v^2 / (2 a) plus one control period of travel).
      double x = 0.0, y = 0.0, th = 0.0;
      double travelled = 0.0;
      double min_clear = std::numeric_limits<double>::infinity();
      bool collide = false;
      std::vector<std::pair<double, double>> traj;
      traj.reserve(steps);
      for (int k = 0; k < steps; ++k) {
        const double nth = th + w * p.sim_dt;
        const double nx = x + v * std::cos(nth) * p.sim_dt;
        const double ny = y + v * std::sin(nth) * p.sim_dt;
        const auto d = sampleDistances(nx, ny, nth);
        for (size_t j = 0; j < d.size(); ++j) {
          if (d[j] < p.margin && d[j] < start_d[j] - 0.01) {
            collide = true;
            break;
          }
        }
        if (collide) {
          break;
        }
        for (const double dj : d) {
          min_clear = std::min(min_clear, dj - p.margin);
        }
        travelled += std::fabs(v) * p.sim_dt;
        x = nx;
        y = ny;
        th = nth;
        traj.emplace_back(x, y);
      }
      if (collide) {
        const double stopping = v * v / (2.0 * p.acc_v) + std::fabs(v) * p.control_period;
        // Already inside the margin: no approach at all, braking or not.
        if (start_inside_margin || travelled < stopping || (v == 0.0 && traj.empty())) {
          continue;
        }
      }
      if (!std::isfinite(min_clear)) {
        min_clear = clearance(0.0, 0.0, 0.0);
      }
      const double dist_to_target = std::hypot(tx - x, ty - y);
      const double heading_err = std::fabs(normalizeAngle(std::atan2(ty - y, tx - x) - th));
      const double clear_cost = 1.0 / std::max(0.02, min_clear + p.margin);
      const double cost = p.w_progress * dist_to_target + p.w_heading * heading_err +
        p.w_clearance * clear_cost - p.w_speed * v;
      if (cost < best_cost) {
        best_cost = cost;
        out.v = v;
        out.w = w;
        out.clearance = min_clear;
        out.trajectory = std::move(traj);
        found = true;
      }
    }
  }
  return found;
}

}  // namespace origin_frontier_explore
