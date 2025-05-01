#pragma once
#include "PathPlannerInterface.h"
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

class AStarPathPlanner : public PathPlannerInterface {
public:
    Trajectory getCollisionFreePath(const OccupancyGrid &grid,
                                    const Eigen::Vector3f &start,
                                    const Eigen::Vector3f &end) override {

        Trajectory path;

        int startX = static_cast<int>(std::floor(start.x() / grid.resolution_m));
        int startY = static_cast<int>(std::floor(start.y() / grid.resolution_m));
        int endX = static_cast<int>(std::floor(end.x() / grid.resolution_m));
        int endY = static_cast<int>(std::floor(end.y() / grid.resolution_m));

        if (endX < 0 || endX >= grid.width || endY < 0 || endY >= grid.height ||
            startX < 0 || startX >= grid.width || startY < 0 || startY >= grid.height ||
            grid.data[grid.get1DIndex(startX, startY)] || grid.data[grid.get1DIndex(endX, endY)]) {
            std::cout << "Invalid start/end point" << std::endl;
            return path;
        }

        auto cmp = [](const std::pair<float, std::pair<int, int>>& a,
                      const std::pair<float, std::pair<int, int>>& b) {
            return a.first > b.first; // Min-heap based on f = g + h
        };

        std::priority_queue<std::pair<float, std::pair<int, int>>,
                            std::vector<std::pair<float, std::pair<int, int>>>,
                            decltype(cmp)> pq(cmp);

        std::vector<bool> visited(grid.width * grid.height, false);
        std::vector<float> g_cost(grid.width * grid.height, std::numeric_limits<float>::max());
        std::vector<std::pair<int, int>> parent(grid.width * grid.height, { -1, -1 });
        std::vector<float> heading(grid.width * grid.height, 0.0f);

        auto heuristic = [&](int x, int y) {
            return std::hypot(endX - x, endY - y);
        };

        pq.push({ heuristic(startX, startY), { startX, startY } });
        g_cost[grid.get1DIndex(startX, startY)] = 0.0f;

        int dx[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
        int dy[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

        bool found = false;
        while (!pq.empty()) {
            auto current = pq.top();
            pq.pop();
            int currentX = current.second.first;
            int currentY = current.second.second;

            if (currentX == endX && currentY == endY) {
                found = true;
                break;
            }

            int currentIndex = grid.get1DIndex(currentX, currentY);
            if (visited[currentIndex]) continue;
            visited[currentIndex] = true;

            for (int i = 0; i < 8; ++i) {
                int nextX = currentX + dx[i];
                int nextY = currentY + dy[i];

                if (nextX >= 0 && nextX < grid.width && nextY >= 0 && nextY < grid.height) {
                    int nextIndex = grid.get1DIndex(nextX, nextY);
                    if (!grid.data[nextIndex]) {
                        float stepCost = std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
                        float newG = g_cost[currentIndex] + stepCost;

                        if (newG < g_cost[nextIndex]) {
                            g_cost[nextIndex] = newG;
                            float f = newG + heuristic(nextX, nextY);
                            pq.push({ f, { nextX, nextY } });
                            parent[nextIndex] = { currentX, currentY };
                            heading[nextIndex] = std::atan2((nextY - currentY), (nextX - currentX));
                        }
                    }
                }
            }
        }

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
            std::reverse(tempPath.begin(), tempPath.end());

            for (const auto& point : tempPath) {
                float x = (point.first + 0.5f) * grid.resolution_m;
                float y = (point.second + 0.5f) * grid.resolution_m;
                float theta = heading[grid.get1DIndex(point.first, point.second)];
                path.push_back(Eigen::Vector3f(x, y, theta));
            }
            path.push_back(end);
            std::cout << "Found A* Path of size: " << path.size() << std::endl;
        } else {
            std::cout << "No valid path found using A*" << std::endl;
        }

        return path;
    }
};
