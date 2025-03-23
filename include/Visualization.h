#pragma once
#include "PathPlannerInterface.h"
#include "matplotlibcpp.h"
#include <iostream>

namespace plt = matplotlibcpp;

struct Visualize{
float _grid_size;
float _x_limit;
float _y_limit;

Visualize(OccupancyGrid &grid) :
  _grid_size(grid.resolution_m),
  _x_limit(grid.width * grid.resolution_m),
  _y_limit(grid.height * grid.resolution_m)
{}

void viewState(Trajectory &path){
  plt::figure_size(1200, 400);

  int n = path.size();
  std::vector<float> x(n), y(n), theta(n);
  for (size_t i = 0; i < n; ++i) {
    x.at(i) = path[i][0];
    y.at(i) = path[i][1];
    theta.at(i) = path[i][2];
  }
  plt::plot(x, {{"color", "red"}});
  plt::named_plot("X vs StepCount", x);

  plt::plot(y, {{"color", "green"}});
  plt::named_plot("Y vs StepCount", y);

  plt::plot(theta, {{"color", "blue"}});
  plt::named_plot("Theta vs StepCount", theta);
  plt::legend();


  plt::tight_layout();
  plt::show();
}

void viewGrid(Trajectory &path, std::vector<std::pair<float, float>> obstacles
){
  plt::figure_size(1200, 1200);
  // Plot obstacles
  for (auto& obstacle: obstacles){
    std::vector<float> corners_x = {
      obstacle.first - _grid_size / 2, // Bottom-left corner
      obstacle.first + _grid_size / 2, // Bottom-right corner
      obstacle.first + _grid_size / 2, // Top-right corner
      obstacle.first - _grid_size / 2  // Top-left corner
    };
    std::vector<float> corners_y = {
      obstacle.second - _grid_size / 2, // Bottom-left corner
      obstacle.second - _grid_size / 2, // Bottom-right corner
      obstacle.second + _grid_size / 2, // Top-right corner
      obstacle.second + _grid_size / 2  // Top-left corner
    };
    plt::fill(corners_x, corners_y, {{"color", "black"}});
    plt::named_plot("Obstacle", corners_x, corners_y);
  }

  int n = path.size();
  std::vector<float> x(n), y(n), theta(n), w(n,2);
  for (size_t i = 0; i < n; ++i) {
    const auto& waypoint = path[i];
      x.at(i) = path[i][0];
      y.at(i) = path[i][1];
      theta.at(i) = path[i][2];
  }
  plt::plot(x, y, {{"color", "blue"}}); 
  plt::named_plot("Path", x, y);
  plt::xlim(0, int(floor(_x_limit)));
  plt::ylim(0, int(floor(_y_limit)));
  plt::grid(true);
  plt::title("Planned Path");
  plt::legend();
  plt::show();
  }
 };