// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0

#include "origin_frontier_explore/grid.hpp"

#include <cmath>
#include <limits>
#include <queue>

namespace origin_frontier_explore
{

bool Grid::worldToCell(double wx, double wy, int & cx, int & cy) const
{
  cx = static_cast<int>(std::floor((wx - origin_x) / resolution));
  cy = static_cast<int>(std::floor((wy - origin_y) / resolution));
  return inBounds(cx, cy);
}

void Grid::cellToWorld(int cx, int cy, double & wx, double & wy) const
{
  wx = origin_x + (cx + 0.5) * resolution;
  wy = origin_y + (cy + 0.5) * resolution;
}

CellState Grid::state(int x, int y, int occupied_threshold) const
{
  const int8_t v = data[index(x, y)];
  if (v < 0) {
    return CellState::kUnknown;
  }
  return v >= occupied_threshold ? CellState::kOccupied : CellState::kFree;
}

std::vector<float> obstacleDistance(
  int width, int height, double resolution, const std::vector<Cell> & obstacles,
  float max_distance)
{
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> dist(static_cast<size_t>(width) * height, inf);
  std::vector<int> source(dist.size(), -1);  // index of the nearest obstacle cell
  std::queue<int> open;

  for (const auto & [x, y] : obstacles) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      continue;
    }
    const int i = y * width + x;
    if (source[i] < 0) {
      dist[i] = 0.0f;
      source[i] = i;
      open.push(i);
    }
  }

  static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (!open.empty()) {
    const int i = open.front();
    open.pop();
    const int x = i % width;
    const int y = i / width;
    const int sx = source[i] % width;
    const int sy = source[i] / width;
    for (int k = 0; k < 8; ++k) {
      const int nx = x + dx[k];
      const int ny = y + dy[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const int n = ny * width + nx;
      const float d = static_cast<float>(std::hypot(nx - sx, ny - sy) * resolution);
      if (d > max_distance || d >= dist[n]) {
        continue;
      }
      dist[n] = d;
      source[n] = source[i];
      open.push(n);
    }
  }
  for (auto & d : dist) {
    if (d > max_distance) {
      d = max_distance;
    }
  }
  return dist;
}

int removeIsolatedObstacles(Grid & grid, int occupied_threshold, int max_neighbours)
{
  std::vector<int> isolated;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (grid.state(x, y, occupied_threshold) != CellState::kOccupied) {
        continue;
      }
      int n = 0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if ((dx || dy) && grid.inBounds(x + dx, y + dy) &&
            grid.state(x + dx, y + dy, occupied_threshold) == CellState::kOccupied)
          {
            ++n;
          }
        }
      }
      if (n <= max_neighbours) {
        isolated.push_back(grid.index(x, y));
      }
    }
  }
  for (const int i : isolated) {
    grid.data[i] = 0;
  }
  return static_cast<int>(isolated.size());
}

std::vector<float> obstacleDistance(const Grid & grid, int occupied_threshold, float max_distance)
{
  std::vector<Cell> obstacles;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      if (grid.state(x, y, occupied_threshold) == CellState::kOccupied) {
        obstacles.emplace_back(x, y);
      }
    }
  }
  return obstacleDistance(grid.width, grid.height, grid.resolution, obstacles, max_distance);
}

}  // namespace origin_frontier_explore
