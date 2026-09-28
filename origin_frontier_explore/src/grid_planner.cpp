// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0

#include "origin_frontier_explore/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace origin_frontier_explore
{

void GridPlanner::setMap(const Grid & grid)
{
  grid_ = grid;
  if (params_.ignore_isolated_obstacles) {
    removeIsolatedObstacles(grid_, params_.occupied_threshold);
  }
  dist_ = obstacleDistance(
    grid_, params_.occupied_threshold, static_cast<float>(params_.inflation_radius));
}

bool GridPlanner::lethal(int x, int y) const
{
  const CellState s = grid_.state(x, y, params_.occupied_threshold);
  if (s == CellState::kOccupied) {
    return true;
  }
  if (s == CellState::kUnknown && !params_.allow_unknown) {
    return true;
  }
  return dist_[grid_.index(x, y)] < params_.robot_radius;
}

double GridPlanner::stepCost(int x, int y) const
{
  double cost = 1.0;
  const double d = dist_[grid_.index(x, y)];
  if (d < params_.inflation_radius) {
    const double t = (params_.inflation_radius - d) /
      (params_.inflation_radius - params_.robot_radius);
    cost += params_.inflation_weight * std::clamp(t, 0.0, 1.0);
  }
  if (grid_.state(x, y, params_.occupied_threshold) == CellState::kUnknown) {
    cost *= params_.unknown_cost;
  }
  return cost;
}

bool GridPlanner::isFree(double wx, double wy) const
{
  int x, y;
  return grid_.worldToCell(wx, wy, x, y) && !lethal(x, y);
}

bool GridPlanner::plan(double sx, double sy, double gx, double gy, Path & path) const
{
  path.clear();
  int x0, y0, x1, y1;
  if (!grid_.worldToCell(sx, sy, x0, y0) || !grid_.worldToCell(gx, gy, x1, y1)) {
    return false;
  }

  // A goal inside inflation (e.g. a frontier next to a wall) moves to the nearest free cell.
  if (lethal(x1, y1)) {
    const int r = static_cast<int>(std::ceil(params_.goal_search_radius / grid_.resolution));
    double best = std::numeric_limits<double>::infinity();
    int bx = -1, by = -1;
    for (int y = y1 - r; y <= y1 + r; ++y) {
      for (int x = x1 - r; x <= x1 + r; ++x) {
        if (grid_.inBounds(x, y) && !lethal(x, y)) {
          const double d = std::hypot(x - x1, y - y1);
          if (d < best && d <= r) {
            best = d;
            bx = x;
            by = y;
          }
        }
      }
    }
    if (bx < 0) {
      return false;
    }
    x1 = bx;
    y1 = by;
  }

  const int start = grid_.index(x0, y0);
  const int goal = grid_.index(x1, y1);
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> g(grid_.data.size(), inf);
  std::vector<int> parent(grid_.data.size(), -1);
  // If the robot starts inside inflation (close to a wall), allow it through inflated cells
  // as long as each step does not get closer to the obstacle.
  std::vector<char> escaping(grid_.data.size(), 0);
  escaping[start] = lethal(x0, y0);
  using Item = std::pair<double, int>;  // (f = g + h, index)
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  auto heuristic = [&](int i) {
      return std::hypot(i % grid_.width - x1, i / grid_.width - y1);
    };

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  g[start] = 0.0;
  open.emplace(heuristic(start), start);
  while (!open.empty()) {
    const auto [f, i] = open.top();
    open.pop();
    if (i == goal) {
      break;
    }
    if (f - heuristic(i) > g[i] + 1e-9) {
      continue;  // stale entry
    }
    const int x = i % grid_.width;
    const int y = i / grid_.width;
    for (int k = 0; k < 8; ++k) {
      const int nx = x + dx[k];
      const int ny = y + dy[k];
      if (!grid_.inBounds(nx, ny)) {
        continue;
      }
      const int n = grid_.index(nx, ny);
      const bool n_lethal = lethal(nx, ny);
      if (n_lethal) {
        const bool occupied =
          grid_.state(nx, ny, params_.occupied_threshold) == CellState::kOccupied;
        if (occupied || !escaping[i] || dist_[n] + 1e-6 < dist_[i]) {
          continue;
        }
      }
      const double ng = g[i] + (k < 4 ? 1.0 : M_SQRT2) * stepCost(nx, ny);
      if (ng < g[n]) {
        g[n] = ng;
        parent[n] = i;
        escaping[n] = n_lethal;
        open.emplace(ng + heuristic(n), n);
      }
    }
  }
  if (g[goal] == inf) {
    return false;
  }

  std::vector<int> cells;
  for (int i = goal; i != -1; i = parent[i]) {
    cells.push_back(i);
  }
  std::reverse(cells.begin(), cells.end());
  for (const int i : cells) {
    double wx, wy;
    grid_.cellToWorld(i % grid_.width, i / grid_.width, wx, wy);
    path.emplace_back(wx, wy);
  }
  return true;
}

}  // namespace origin_frontier_explore
