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

void viewGrid(Trajectory &path, std::vector<Eigen::Vector4f> obstacles
){
  plt::figure_size(1200, 1200);
  // Plot obstacles
  for (auto& obstacle: obstacles){
    std::vector<float> corners_x = {
      obstacle[0] - obstacle[2] / 2, // Bottom-left corner
      obstacle[0] + obstacle[2] / 2, // Bottom-right corner
      obstacle[0] + obstacle[2] / 2, // Top-right corner
      obstacle[0] - obstacle[2] / 2  // Top-left corner
    };
    std::vector<float> corners_y = {
      obstacle[1] - obstacle[3] / 2, // Bottom-left corner
      obstacle[1] - obstacle[3] / 2, // Bottom-right corner
      obstacle[1] + obstacle[3] / 2, // Top-right corner
      obstacle[1] + obstacle[3] / 2  // Top-left corner
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
  // Mark the start of the path with a circle
  if (!x.empty() && !y.empty()) {
      std::vector<float> start_x = {x[0]}; // Wrap start x-coordinate in a vector
      std::vector<float> start_y = {y[0]}; // Wrap start y-coordinate in a vector
      plt::scatter(start_x, start_y, 250.0, {{"marker", "o"}, {"color", "green"}}); // Circle marker at start
      plt::named_plot("Start", start_x, start_y);
  }

  // Mark the end of the path with a star
  if (!x.empty() && !y.empty()) {
      std::vector<float> end_x = {x.back()}; // Wrap end x-coordinate in a vector
      std::vector<float> end_y = {y.back()}; // Wrap end y-coordinate in a vector
      plt::scatter(end_x, end_y, 400.0, {{"marker", "*"}, {"color", "red"}}); // Star marker at end
      plt::named_plot("End", end_x, end_y);
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