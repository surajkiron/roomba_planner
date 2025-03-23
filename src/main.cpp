#include <iostream>
#include <memory>
#include "BFSPathPlanner.h"
#include "DijkstraPathPlanner.h"
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "Visualization.h"



int main() {

  OccupancyGrid grid{};
  Visualize vis(grid);
  Planner type = Planner::BFS;

  std::unique_ptr<PathPlannerInterface> planner;
  switch (type) {
    case Planner::BFS:
      planner = std::make_unique<BFSPathPlanner>();
      break;
    case Planner::Dijkstra:
      planner = std::make_unique<DijkstraPathPlanner>();
      break;
    default:
      std::cerr << "Unknown planner type" << std::endl;
      return -1;
  }
  std::vector<Eigen::Vector4f> obstacles;
  obstacles = {
    {1.0, 10.0, 2.0, 1.0},
    {11.0, 11.0, 5.0, 5.0},
  };
  for (auto& obstacle : obstacles){
    grid.setAsObstacleWithDimension(obstacle[0], obstacle[1], obstacle[2], obstacle[3]);
  }

  auto start = Eigen::Vector3f{0.50, 0.50, 0.0};
  auto end = Eigen::Vector3f{1.0, 15.0, 0.0};
  Trajectory path = planner->getCollisionFreePath(grid, start, end);
  // print function for vector
  int n = path.size();
  std::vector<float> x(n), y(n), theta(n), w(n,2);
  for (size_t i = 0; i < n; ++i) {
    const auto& waypoint = path[i];
      x.at(i) = path[i][0];
      y.at(i) = path[i][1];
      theta.at(i) = path[i][2];
    std::cout << waypoint.transpose() << std::endl;
  }
  vis.viewGrid(path, obstacles);
  vis.viewState(path);
  return 0;
}
