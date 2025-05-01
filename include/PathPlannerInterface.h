#pragma once
#include <Eigen/Dense>
#include "OccupancyGrid.h"
#include <vector>
#include <queue>
#include <numbers>

typedef std::vector<Eigen::Vector3f> Trajectory;
constexpr double pi = 3.14159265358979323846;

class PathPlannerInterface {

public:
  virtual Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                          const Eigen::Vector3f &start,
                                          const Eigen::Vector3f &end) = 0;

};
enum class Planner {
  BFS,
  Dijkstra,
  AStar,
};

