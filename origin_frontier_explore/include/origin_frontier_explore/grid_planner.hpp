// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// A* global planner on a 2D occupancy grid with obstacle inflation.
//
// Cells closer than `robot_radius` to an obstacle are lethal. Between `robot_radius` and
// `inflation_radius` a cost penalty pushes the path towards the middle of free space.
// Unknown cells may be crossed at extra cost (frontier goals lie next to unknown space).

#pragma once

#include <utility>
#include <vector>

#include "origin_frontier_explore/grid.hpp"

namespace origin_frontier_explore
{

struct PlannerParams
{
  int occupied_threshold{65};
  double robot_radius{0.35};       // m, lethal distance to obstacles
  double inflation_radius{0.9};    // m, penalty beyond robot_radius up to this distance
  double inflation_weight{4.0};    // penalty multiplier at robot_radius
  bool allow_unknown{true};
  double unknown_cost{3.0};        // step cost multiplier through unknown cells
  double goal_search_radius{0.5};  // m, move a lethal goal to the nearest free cell
  bool ignore_isolated_obstacles{true};  // see removeIsolatedObstacles(); local DWA still sees them
};

using Path = std::vector<std::pair<double, double>>;  // world (x, y)

class GridPlanner
{
public:
  explicit GridPlanner(const PlannerParams & params = PlannerParams())
  : params_(params) {}

  // Precompute obstacle distances for `grid`. Call once per map update.
  void setMap(const Grid & grid);

  // Plan from start to goal (world coordinates). Returns false if no path exists.
  bool plan(double sx, double sy, double gx, double gy, Path & path) const;

  // True if the world point is at least robot_radius from any obstacle.
  bool isFree(double wx, double wy) const;

  const Grid & grid() const {return grid_;}

private:
  bool lethal(int x, int y) const;
  double stepCost(int x, int y) const;

  PlannerParams params_;
  Grid grid_;
  std::vector<float> dist_;
};

}  // namespace origin_frontier_explore
