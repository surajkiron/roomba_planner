#include <iostream>
#include <memory>
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "BFSPathPlanner.h"
#include "Visualization.h"



int main() {

  OccupancyGrid grid{};
  Visualize vis(grid);

  std::unique_ptr<PathPlannerInterface> planner = std::make_unique<BFSPathPlanner>();
  std::vector<std::pair<float, float>> obstacles;
  obstacles = {
    {10.0, 10.0},
    {11.0, 11.0},
  };
  for (auto& obstacle : obstacles){
    grid.setAsObstacle(obstacle.first, obstacle.second);
  }

  auto start = Eigen::Vector3f{0.50, 0.50, 0.0};
  auto end = Eigen::Vector3f{15.0, 15.0, 0.0};
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
  return 0;
}
