// Copyright 2026 Soumic Sarkar
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "origin_frontier_explore/dwa_controller.hpp"
#include "origin_frontier_explore/frontier_search.hpp"
#include "origin_frontier_explore/grid_planner.hpp"

using origin_frontier_explore::DwaCommand;
using origin_frontier_explore::DwaController;
using origin_frontier_explore::findFrontiers;
using origin_frontier_explore::FrontierSearchParams;
using origin_frontier_explore::Grid;
using origin_frontier_explore::GridPlanner;
using origin_frontier_explore::Path;

namespace
{
// 10 x 6 m grid at 0.05 m, origin (0, 0). Value v everywhere.
Grid makeGrid(int8_t v)
{
  Grid g;
  g.width = 200;
  g.height = 120;
  g.resolution = 0.05;
  g.data.assign(static_cast<size_t>(g.width) * g.height, v);
  return g;
}

void fillRect(Grid & g, double x0, double y0, double x1, double y1, int8_t v)
{
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width; ++x) {
      double wx, wy;
      g.cellToWorld(x, y, wx, wy);
      if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) {
        g.data[g.index(x, y)] = v;
      }
    }
  }
}
}  // namespace

TEST(FrontierSearch, FindsBoundaryOfKnownArea)
{
  // Known free 4 m square in the middle of unknown space, robot inside.
  Grid g = makeGrid(-1);
  fillRect(g, 2.0, 1.0, 6.0, 5.0, 0);
  const auto f = findFrontiers(g, 4.0, 3.0, FrontierSearchParams());
  ASSERT_FALSE(f.empty());
  // All four sides of the square are one connected frontier ring.
  EXPECT_EQ(f.size(), 1u);
  EXPECT_NEAR(f[0].centroid_x, 4.0, 0.2);
  EXPECT_GT(f[0].length, 15.0);  // all four sides
}

TEST(FrontierSearch, IgnoresUnreachableAndWalledOffFrontiers)
{
  // Free corridor 0..9 x 2..4, walled in on the long sides; open end at x = 9 to unknown.
  Grid g = makeGrid(-1);
  fillRect(g, 0.0, 1.8, 9.0, 4.2, 100);
  fillRect(g, 0.2, 2.0, 9.0, 4.0, 0);  // closed at x = 0 by the wall
  // Small isolated free pocket touching unknown, unreachable from the corridor.
  fillRect(g, 1.0, 5.0, 2.0, 5.5, 0);
  const auto f = findFrontiers(g, 1.0, 3.0, FrontierSearchParams());
  ASSERT_EQ(f.size(), 1u);
  EXPECT_GT(f[0].goal_x, 8.5);  // the open end of the corridor
  EXPECT_NEAR(f[0].goal_y, 3.0, 0.3);
  EXPECT_NEAR(f[0].travel_distance, 8.0, 0.5);
}

TEST(FrontierSearch, ScatteredSingleObstaclesDoNotBlockTheWay)
{
  // Corridor 1.2 m wide, open to unknown at x = 9.8; single-cell "clutter" returns every 1 m
  // on its centre line. With 0.40 m clearance each would block the corridor completely.
  Grid g = makeGrid(100);
  fillRect(g, 0.2, 2.4, 9.8, 3.6, 0);
  fillRect(g, 9.8, 2.4, 10.0, 3.6, -1);
  for (double x = 2.0; x < 9.0; x += 1.0) {
    int cx, cy;
    g.worldToCell(x, 3.0, cx, cy);
    g.data[g.index(cx, cy)] = 100;
  }
  FrontierSearchParams p;
  EXPECT_EQ(findFrontiers(g, 1.0, 3.0, p).size(), 1u);
  p.ignore_isolated_obstacles = false;
  EXPECT_TRUE(findFrontiers(g, 1.0, 3.0, p).empty());
}

TEST(FrontierSearch, NoFrontiersInClosedRoom)
{
  Grid g = makeGrid(100);
  fillRect(g, 1.0, 1.0, 5.0, 5.0, 0);
  EXPECT_TRUE(findFrontiers(g, 3.0, 3.0, FrontierSearchParams()).empty());
}

TEST(FrontierSearch, PrefersCloserFrontierOfSimilarSize)
{
  // Corridor open to unknown at both ends; robot near the left end.
  Grid g = makeGrid(100);
  fillRect(g, 0.0, 2.0, 10.0, 4.0, 0);
  fillRect(g, 0.0, 2.0, 0.2, 4.0, -1);
  fillRect(g, 9.8, 2.0, 10.0, 4.0, -1);
  const auto f = findFrontiers(g, 2.0, 3.0, FrontierSearchParams());
  ASSERT_EQ(f.size(), 2u);
  EXPECT_LT(f[0].goal_x, 1.0);
}

TEST(GridPlanner, PlansThroughDoorAroundWall)
{
  // Wall at x = 5 with a 1.5 m door at y = 2.25..3.75.
  Grid g = makeGrid(0);
  fillRect(g, 4.9, 0.0, 5.1, 6.0, 100);
  fillRect(g, 4.9, 2.25, 5.1, 3.75, 0);
  GridPlanner planner;
  planner.setMap(g);
  Path path;
  ASSERT_TRUE(planner.plan(1.0, 1.0, 9.0, 1.0, path));
  bool through_door = false;
  for (const auto & [x, y] : path) {
    if (std::fabs(x - 5.0) < 0.1) {
      through_door = y > 2.25 && y < 3.75;
    }
    EXPECT_TRUE(planner.isFree(x, y) || std::hypot(x - 1.0, y - 1.0) < 0.5);
  }
  EXPECT_TRUE(through_door);
  EXPECT_NEAR(path.back().first, 9.0, 0.1);
}

TEST(GridPlanner, FailsWhenGoalIsWalledOff)
{
  Grid g = makeGrid(0);
  fillRect(g, 4.9, 0.0, 5.1, 6.0, 100);
  GridPlanner planner;
  planner.setMap(g);
  Path path;
  EXPECT_FALSE(planner.plan(1.0, 3.0, 9.0, 3.0, path));
}

TEST(GridPlanner, EscapesWhenStartIsInsideInflation)
{
  Grid g = makeGrid(0);
  fillRect(g, 0.0, 0.0, 10.0, 0.5, 100);
  GridPlanner planner;
  planner.setMap(g);
  Path path;
  EXPECT_TRUE(planner.plan(2.0, 0.7, 8.0, 4.0, path));  // start 0.2 m from the wall
}

TEST(Dwa, DrivesStraightWhenClear)
{
  DwaController dwa;
  dwa.setObstacles({});
  DwaCommand cmd;
  ASSERT_TRUE(dwa.compute(0.3, 0.0, 3.0, 0.0, cmd));
  EXPECT_GT(cmd.v, 0.3);
  EXPECT_NEAR(cmd.w, 0.0, 0.15);
}

TEST(Dwa, SteersAroundObstacleTowardsPathTarget)
{
  // Wall segment 0.9 m ahead, open to the left; the global path's lookahead point is to the
  // front-left, past the wall's end.
  std::vector<std::pair<double, double>> obs;
  for (double y = -1.0; y <= 0.3; y += 0.05) {
    obs.emplace_back(0.9, y);
  }
  DwaController dwa;
  dwa.setObstacles(obs);
  DwaCommand cmd;
  ASSERT_TRUE(dwa.compute(0.2, 0.0, 1.2, 1.2, cmd));
  EXPECT_GT(cmd.clearance, 0.0);  // the whole rolled-out arc stays clear of the wall
  EXPECT_GT(cmd.w, 0.0);          // turns left, towards the open side
}

TEST(Dwa, BrakesWhenObstacleIsWithinStoppingDistance)
{
  // Wall 0.5 m ahead: the front of the footprint is 0.11 m from the margin, less than the
  // 0.12 m needed to stop from 0.4 m/s, so full speed is no longer admissible.
  std::vector<std::pair<double, double>> obs;
  for (double y = -1.0; y <= 1.0; y += 0.05) {
    obs.emplace_back(0.5, y);
  }
  DwaController dwa;
  dwa.setObstacles(obs);
  DwaCommand cmd;
  ASSERT_TRUE(dwa.compute(0.4, 0.0, 3.0, 0.0, cmd));
  EXPECT_LT(cmd.v, 0.4);
}

TEST(Dwa, KeepsFullSpeedWhileObstacleIsFarEnoughToBrake)
{
  std::vector<std::pair<double, double>> obs;
  for (double y = -1.0; y <= 1.0; y += 0.05) {
    obs.emplace_back(1.5, y);
  }
  DwaController dwa;
  dwa.setObstacles(obs);
  DwaCommand cmd;
  ASSERT_TRUE(dwa.compute(0.4, 0.0, 3.0, 0.0, cmd));
  EXPECT_GT(cmd.v, 0.3);
}

TEST(Dwa, RotatesInPlaceTowardsTargetBehind)
{
  DwaController dwa;
  dwa.setObstacles({});
  DwaCommand cmd;
  ASSERT_TRUE(dwa.compute(0.0, 0.0, -2.0, 0.5, cmd));
  EXPECT_GT(std::fabs(cmd.w), 0.2);
}

TEST(Dwa, DoesNotPushFurtherIntoObstacleAlreadyInsideMargin)
{
  // Wall 3 cm in front of the footprint (inside the 6 cm margin): the robot may turn or stay,
  // but not drive on into it.
  std::vector<std::pair<double, double>> obs;
  for (double y = -1.0; y <= 1.0; y += 0.02) {
    obs.emplace_back(0.36, y);
  }
  DwaController dwa;
  dwa.setObstacles(obs);
  DwaCommand cmd;
  if (dwa.compute(0.0, 0.0, 3.0, 0.0, cmd)) {
    EXPECT_LT(cmd.v, 0.05);
  }
}
