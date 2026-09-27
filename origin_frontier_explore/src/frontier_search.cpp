// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0

#include "origin_frontier_explore/frontier_search.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace origin_frontier_explore
{

namespace
{
const int kDx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
const int kDy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
}  // namespace

std::vector<Frontier> findFrontiers(
  const Grid & grid, double robot_x, double robot_y, const FrontierSearchParams & params)
{
  std::vector<Frontier> frontiers;
  if (grid.width == 0 || grid.height == 0) {
    return frontiers;
  }

  const float clearance = static_cast<float>(params.robot_clearance);
  const auto dist = obstacleDistance(grid, params.occupied_threshold, clearance + 0.1f);
  auto traversable = [&](int x, int y) {
      return grid.state(x, y, params.occupied_threshold) == CellState::kFree &&
             dist[grid.index(x, y)] >= clearance;
    };

  // Start from the robot cell, or the nearest traversable cell if the robot sits in inflation.
  int rx, ry;
  if (!grid.worldToCell(robot_x, robot_y, rx, ry)) {
    return frontiers;
  }
  if (!traversable(rx, ry)) {
    const int r = static_cast<int>(std::ceil(params.start_search_radius / grid.resolution));
    double best = std::numeric_limits<double>::infinity();
    int bx = -1, by = -1;
    for (int y = ry - r; y <= ry + r; ++y) {
      for (int x = rx - r; x <= rx + r; ++x) {
        if (grid.inBounds(x, y) && traversable(x, y)) {
          const double d = std::hypot(x - rx, y - ry);
          if (d < best && d <= r) {
            best = d;
            bx = x;
            by = y;
          }
        }
      }
    }
    if (bx < 0) {
      return frontiers;
    }
    rx = bx;
    ry = by;
  }

  // Dijkstra over traversable cells: travel distance from the robot.
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> travel(grid.data.size(), inf);
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  travel[grid.index(rx, ry)] = 0.0;
  open.emplace(0.0, grid.index(rx, ry));
  while (!open.empty()) {
    const auto [d, i] = open.top();
    open.pop();
    if (d > travel[i]) {
      continue;
    }
    const int x = i % grid.width;
    const int y = i / grid.width;
    for (int k = 0; k < 8; ++k) {
      const int nx = x + kDx8[k];
      const int ny = y + kDy8[k];
      if (!grid.inBounds(nx, ny) || !traversable(nx, ny)) {
        continue;
      }
      const double nd = d + (k < 4 ? 1.0 : M_SQRT2) * grid.resolution;
      const int n = grid.index(nx, ny);
      if (nd < travel[n]) {
        travel[n] = nd;
        open.emplace(nd, n);
      }
    }
  }

  // Frontier cells: reachable cells with an unknown 4-neighbour.
  std::vector<char> is_frontier(grid.data.size(), 0);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (travel[grid.index(x, y)] == inf) {
        continue;
      }
      for (int k = 0; k < 4; ++k) {
        const int nx = x + kDx8[k];
        const int ny = y + kDy8[k];
        // Outside the grid counts as unknown: SLAM maps only extend as far as known space.
        if (!grid.inBounds(nx, ny) ||
          grid.state(nx, ny, params.occupied_threshold) == CellState::kUnknown)
        {
          is_frontier[grid.index(x, y)] = 1;
          break;
        }
      }
    }
  }

  // Cluster (8-connected) and score.
  std::vector<char> visited(grid.data.size(), 0);
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const int start = grid.index(x, y);
      if (!is_frontier[start] || visited[start]) {
        continue;
      }
      Frontier f;
      std::queue<int> q;
      q.push(start);
      visited[start] = 1;
      while (!q.empty()) {
        const int i = q.front();
        q.pop();
        const int cx = i % grid.width;
        const int cy = i / grid.width;
        f.cells.emplace_back(cx, cy);
        for (int k = 0; k < 8; ++k) {
          const int nx = cx + kDx8[k];
          const int ny = cy + kDy8[k];
          if (!grid.inBounds(nx, ny)) {
            continue;
          }
          const int n = grid.index(nx, ny);
          if (is_frontier[n] && !visited[n]) {
            visited[n] = 1;
            q.push(n);
          }
        }
      }
      if (static_cast<int>(f.cells.size()) < params.min_frontier_cells) {
        continue;
      }

      double sx = 0.0, sy = 0.0;
      for (const auto & [cx, cy] : f.cells) {
        double wx, wy;
        grid.cellToWorld(cx, cy, wx, wy);
        sx += wx;
        sy += wy;
      }
      f.centroid_x = sx / f.cells.size();
      f.centroid_y = sy / f.cells.size();

      // Goal: the frontier cell closest to the centroid (always reachable and free).
      double best = inf;
      for (const auto & [cx, cy] : f.cells) {
        double wx, wy;
        grid.cellToWorld(cx, cy, wx, wy);
        const double d = std::hypot(wx - f.centroid_x, wy - f.centroid_y);
        if (d < best) {
          best = d;
          f.goal_x = wx;
          f.goal_y = wy;
          f.travel_distance = travel[grid.index(cx, cy)];
        }
      }
      f.length = f.cells.size() * grid.resolution;
      f.score = params.size_weight * f.length - params.distance_weight * f.travel_distance;
      frontiers.push_back(std::move(f));
    }
  }

  std::sort(
    frontiers.begin(), frontiers.end(),
    [](const Frontier & a, const Frontier & b) {return a.score > b.score;});
  return frontiers;
}

}  // namespace origin_frontier_explore
