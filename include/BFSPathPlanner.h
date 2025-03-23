#pragma once
#include "PathPlannerInterface.h"
#include "iostream"

class BFSPathPlanner : public PathPlannerInterface {
public:
  Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                  const Eigen::Vector3f &start,
                                  const Eigen::Vector3f &end){

    Trajectory path;
    // Convert start and end positions to grid indices
    int startX = static_cast<int>(std::floor(start.x() / grid.resolution_m));
    int startY = static_cast<int>(std::floor(start.y() / grid.resolution_m));
    int endX = static_cast<int>(std::floor(end.x() / grid.resolution_m));
    int endY = static_cast<int>(std::floor(end.y() / grid.resolution_m));

    // Check if start or end is out of bounds or in an obstacle
    if (endX < 0 || endX >= grid.width || endY < 0 || endY >= grid.height){
      std::cout<<"End is out of bounds, make sure it's in range (0.0, "<<grid.height*grid.resolution_m<<")";
      return path;
    }
    if (startX < 0 || startX >= grid.width || startY < 0 || startY >= grid.height){
      std::cout<<"Start is out of bounds, make sure it's in range (0.0, "<<grid.height*grid.resolution_m<<")";
      return path;
    }
    if (grid.data[grid.get1DIndex(startX, startY)] || grid.data[grid.get1DIndex(endX, endY)]
        ){
          std::cout<<"Infeasible start/endpoint "<<std::endl;
          return path; // Return empty path if start or end is invalid
        }

    // BFS queue and visited set
    std::queue<std::pair<int, int>> queue;
    std::vector<bool> visited = std::vector<bool>(320*320,false);
    std::vector<std::pair<int, int>> parent(grid.width * grid.height, {startX, startY}); // ???

    // Start BFS
    queue.push({startX, startY});
    visited[grid.get1DIndex(startX, startY)]=true;

    // Define 8-connected neighborhood
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    float dtheta[] = {-3*pi/4, pi, 3*pi/4, -pi/2, pi/2, -pi/4, 0.0, pi/4};

    bool found = false;
    while (!queue.empty()) {
      auto current = queue.front();
      queue.pop();
      int currentX = current.first;
      int currentY = current.second;

      if (currentX == endX && currentY == endY) {
          found = true;
          break;
      }
      for (int i = 0; i < 8; ++i) {
        int nextX = currentX + dx[i];
        int nextY = currentY + dy[i];

        if (nextX >= 0 && nextX < grid.width && nextY >= 0 && nextY < grid.height) {
          int nextIndex = grid.get1DIndex(nextX, nextY);
          if (!grid.data[nextIndex] && !visited[nextIndex]) {
            queue.push({nextX, nextY});
            visited[nextIndex]=true;
            parent[nextIndex] = {currentX, currentY};
          }
        }
      }
    }
    // Reconstruct path if found
    if (found) {
      std::vector<std::pair<int, int>> tempPath;
      int currentX = endX;
      int currentY = endY;

      while (currentX != startX || currentY != startY) {
        tempPath.push_back({currentX, currentY});
        int currentIndex = grid.get1DIndex(currentX, currentY);
        currentX = parent[currentIndex].first;
        currentY = parent[currentIndex].second;
      }
      tempPath.push_back({startX, startY});
      // Reverse to get start to end order
      std::reverse(tempPath.begin(), tempPath.end());

      // Convert grid indices back to coordinates
      for (const auto& point : tempPath) {
        float x = (point.first + 0.5f) * grid.resolution_m;
        float y = (point.second + 0.5f) * grid.resolution_m;
        path.push_back(Eigen::Vector3f(x, y, 0.0f)); // Assuming theta is 0 for simplicity
      }
    }
    return path;
  }
};