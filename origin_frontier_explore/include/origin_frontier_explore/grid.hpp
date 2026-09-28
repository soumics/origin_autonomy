// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0
//
// Minimal occupancy-grid view shared by the frontier search and the grid planner.
// Independent of ROS messages so the algorithms can be unit-tested directly.

#pragma once

#include <cstdint>
#include <utility>
#include <vector>

namespace origin_frontier_explore
{

using Cell = std::pair<int, int>;  // (x, y) cell indices

enum class CellState { kFree, kOccupied, kUnknown };

struct Grid
{
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};  // world position of cell (0, 0)'s lower-left corner
  double origin_y{0.0};
  std::vector<int8_t> data;  // row-major, -1 unknown, 0..100 occupancy probability

  bool inBounds(int x, int y) const {return x >= 0 && y >= 0 && x < width && y < height;}
  int index(int x, int y) const {return y * width + x;}
  bool worldToCell(double wx, double wy, int & cx, int & cy) const;
  void cellToWorld(int cx, int cy, double & wx, double & wy) const;  // cell centre
  CellState state(int x, int y, int occupied_threshold) const;
};

// Occupied cells with at most `max_neighbours` occupied 8-neighbours (single lidar returns from
// clutter, people, sensor noise) set to free. Real maps of a real Origin One are sprinkled with
// them; with the robot's 0.40 m clearance each one blocks a 0.8 m circle and cut every path to
// the frontiers. Walls and objects always have occupied neighbours. Returns the count removed.
int removeIsolatedObstacles(Grid & grid, int occupied_threshold, int max_neighbours = 1);

// Distance (m) from every cell to the nearest occupied cell, capped at max_distance.
// Unknown cells are not obstacles. Brushfire propagation of the nearest obstacle cell.
std::vector<float> obstacleDistance(const Grid & grid, int occupied_threshold, float max_distance);

// Same, for an arbitrary set of obstacle cells on a width x height grid.
std::vector<float> obstacleDistance(
  int width, int height, double resolution, const std::vector<Cell> & obstacles,
  float max_distance);

}  // namespace origin_frontier_explore
