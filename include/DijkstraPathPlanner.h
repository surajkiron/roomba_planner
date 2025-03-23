#pragma once
#include "PathPlannerInterface.h"
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

class DijkstraPathPlanner : public PathPlannerInterface {
public:
    Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                    const Eigen::Vector3f &start,
                                    const Eigen::Vector3f &end) override {

        Trajectory path;

        // Convert start and end positions to grid indices
        int startX = static_cast<int>(std::floor(start.x() / grid.resolution_m));
        int startY = static_cast<int>(std::floor(start.y() / grid.resolution_m));
        int endX = static_cast<int>(std::floor(end.x() / grid.resolution_m));
        int endY = static_cast<int>(std::floor(end.y() / grid.resolution_m));

        // Check if start or end is out of bounds or in an obstacle
        if (endX < 0 || endX >= grid.width || endY < 0 || endY >= grid.height) {
            std::cout << "End is out of bounds, make sure it's in range (0.0, " << grid.height * grid.resolution_m << ")";
            return path;
        }
        if (startX < 0 || startX >= grid.width || startY < 0 || startY >= grid.height) {
            std::cout << "Start is out of bounds, make sure it's in range (0.0, " << grid.height * grid.resolution_m << ")";
            return path;
        }
        if (grid.data[grid.get1DIndex(startX, startY)] || grid.data[grid.get1DIndex(endX, endY)]) {
            std::cout << "Infeasible start/endpoint " << std::endl;
            return path; // Return empty path if start or end is invalid
        }

        // Priority queue for Dijkstra's algorithm
        auto cmp = [](const std::pair<float, std::pair<int, int>>& a, const std::pair<float, std::pair<int, int>>& b) {
            return a.first > b.first; // Min-heap based on cost
        };
        std::priority_queue<std::pair<float, std::pair<int, int>>, std::vector<std::pair<float, std::pair<int, int>>>, decltype(cmp)> pq(cmp);

        // Visited set and cost tracking
        std::vector<bool> visited(grid.width * grid.height, false);
        std::vector<float> cost(grid.width * grid.height, std::numeric_limits<float>::max());
        std::vector<std::pair<int, int>> parent(grid.width * grid.height, { -1, -1 });
        std::vector<float> heading(grid.width * grid.height, 0.0f);

        // Initialize start node
        pq.push({ 0.0f, { startX, startY } });
        cost[grid.get1DIndex(startX, startY)] = 0.0f;

        // Define 8-connected neighborhood
        int dx[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
        int dy[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

        bool found = false;
        while (!pq.empty()) {
            auto current = pq.top();
            pq.pop();
            float currentCost = current.first;
            int currentX = current.second.first;
            int currentY = current.second.second;

            if (currentX == endX && currentY == endY) {
                found = true;
                break;
            }

            if (visited[grid.get1DIndex(currentX, currentY)]) continue;
            visited[grid.get1DIndex(currentX, currentY)] = true;

            for (int i = 0; i < 8; ++i) {
                int nextX = currentX + dx[i];
                int nextY = currentY + dy[i];

                if (nextX >= 0 && nextX < grid.width && nextY >= 0 && nextY < grid.height) {
                    int nextIndex = grid.get1DIndex(nextX, nextY);
                    if (!grid.data[nextIndex]) {
                        float newCost = currentCost + std::hypot(dx[i], dy[i]); // Euclidean distance as cost
                        if (newCost < cost[nextIndex]) {
                            cost[nextIndex] = newCost;
                            pq.push({ newCost, { nextX, nextY } });
                            parent[nextIndex] = { currentX, currentY };
                            heading[nextIndex] = atan2((nextY - currentY), (nextX - currentX));
                        }
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
                tempPath.push_back({ currentX, currentY });
                int currentIndex = grid.get1DIndex(currentX, currentY);
                currentX = parent[currentIndex].first;
                currentY = parent[currentIndex].second;
            }
            tempPath.push_back({ startX, startY });

            // Reverse to get start to end order
            std::reverse(tempPath.begin(), tempPath.end());

            // Convert grid indices back to coordinates
            for (const auto& point : tempPath) {
                float x = (point.first + 0.5f) * grid.resolution_m;
                float y = (point.second + 0.5f) * grid.resolution_m;
                float theta = heading[grid.get1DIndex(point.first, point.second)];
                path.push_back(Eigen::Vector3f(x, y, theta));
            }
        }

        return path;
    }
};