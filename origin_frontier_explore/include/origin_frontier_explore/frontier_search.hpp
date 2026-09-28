// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Frontier detection on a 2D occupancy grid.
//
// A frontier cell is a free cell, reachable from the robot through free space with enough
// clearance for the robot body, that touches unknown space (4-neighbourhood). Frontier cells
// are grouped into 8-connected clusters; each cluster gets a goal (its cell closest to the
// cluster centroid), the travel distance to that goal through free space (Dijkstra), and a
// score that prefers large frontiers that are close by.

#pragma once

#include <vector>

#include "origin_frontier_explore/grid.hpp"

namespace origin_frontier_explore
{

struct FrontierSearchParams
{
  int occupied_threshold{65};
  double robot_clearance{0.35};   // m, min distance to obstacles for a traversable cell
  int min_frontier_cells{8};      // smaller clusters are sensor noise
  double size_weight{1.0};        // score += size_weight * frontier length (m)
  double distance_weight{1.0};    // score -= distance_weight * travel distance (m)
  double start_search_radius{1.0};  // m, if the robot cell itself is not traversable
  bool ignore_isolated_obstacles{true};  // see removeIsolatedObstacles()
};

struct Frontier
{
  std::vector<Cell> cells;
  double centroid_x{0.0};
  double centroid_y{0.0};
  double goal_x{0.0};         // world coordinates of the navigation goal
  double goal_y{0.0};
  double length{0.0};         // cells * resolution (m)
  double travel_distance{0.0};  // m through free space from the robot
  double score{0.0};
};

// Returns frontiers sorted by descending score. Empty if none are reachable.
std::vector<Frontier> findFrontiers(
  const Grid & grid, double robot_x, double robot_y, const FrontierSearchParams & params);

}  // namespace origin_frontier_explore
