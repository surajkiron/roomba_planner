#include <iostream>
#include <memory>
#include "AStarPathPlanner.h"
#include "BFSPathPlanner.h"
#include "ConfigParser.h"
#include "DijkstraPathPlanner.h"
#include "OccupancyGrid.h"
#include "PathPlannerInterface.h"
#include "Visualization.h"


int main() {

  OccupancyGrid grid{};
  Visualize vis(grid);
  Planner type = Planner::BFS;
  ConfigParser config("./config.yml");

  std::unique_ptr<PathPlannerInterface> planner;
  switch (config.planner) {
    case Planner::BFS:
      planner = std::make_unique<BFSPathPlanner>();
      break;
    case Planner::Dijkstra:
      planner = std::make_unique<DijkstraPathPlanner>();
      break;
    case Planner::AStar:
      planner = std::make_unique<AStarPathPlanner>();
      break;
    default:
      std::cerr << "Unknown planner type" << std::endl;
      return -1;
  }
  std::vector<Eigen::Vector4f> obstacles = config.
    occupancyGrid.obstacles;
  for (auto& obstacle : obstacles){
    grid.setAsObstacleWithDimension(obstacle[0], obstacle[1], obstacle[2], obstacle[3]);
  }

  auto start = config.start;
  auto end = config.end;
  Trajectory path = planner->getCollisionFreePath(grid, start, end);
  // print function for vector
  int n = path.size();
  std::vector<float> x(n), y(n), theta(n), w(n,2);
 
  vis.viewGrid(path, obstacles);
  vis.viewState(path);
  return 0;
}
