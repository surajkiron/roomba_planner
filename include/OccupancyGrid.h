#pragma once
#include <cmath>
#include <vector>
#include <cmath>

/**
 * @struct OccupancyGrid struct contains the grid, its dimensions and resolution
 */
struct OccupancyGrid {
  int width = 320; ///< Number of cells along the width(x axis)
  int height = 320; ///< Number of cells along the height(yxis)
  float resolution_m = 0.05; ///< size of each cell in meters
  std::vector<bool> data = std::vector<bool>(320*320,false); ///< The bin occ grid in with origin at index 0 and row major format

  /**
   * @brief Given a 2d cell index in 0 based indexing, returns the 1d index in the row major grid
   * @returns the 1d index in 0 based indexing. -1 if given index is out of the grid
   * @param xIndex is an integer denoting the index along the x direction in the grid
   * @param yIndex is an integer denoting the index along the y direction in the grid
   */
  int get1DIndex(int xIndex, int yIndex) const {
    int result = yIndex * width + xIndex;
    return (result>= (width * height))? -1: result;
  }

  /**
   * @brief Given a coordinate, we set the coordinate as an obstacle
   * @param xCoordinate is the x coordinate in m
   * @param yCoordinate is the y coordinate in m
   */
  void setAsObstacle(float xCoordinate, float yCoordinate) {
    int xIndex = int(std::floor(xCoordinate)/resolution_m);
    int yIndex = int(std::floor(yCoordinate)/resolution_m);

    int obsIndex = get1DIndex(xIndex, yIndex);

    if (obsIndex>=0 && obsIndex < width * height)
        data[obsIndex] = true;
  }

  /**
   * @brief Given a coordinate, we set the coordinate as an obstacle
   * @param xCoordinate is the x coordinate in m
   * @param yCoordinate is the y coordinate in m
   * @param obstacle_height is the size in y direction
   * @param obstacle_width is the size in x direction
   */
  void setAsObstacleWithDimension(float xCoordinate, float yCoordinate, float obstacle_width, float obstacle_height) {
    int xIndex = int(std::floor(xCoordinate)/resolution_m);
    int yIndex = int(std::floor(yCoordinate)/resolution_m);
    int cell_width = int(std::floor(obstacle_width)/resolution_m);
    int cell_height = int(std::floor(obstacle_height)/resolution_m);

    std::vector<float> corners_x = {
      xIndex - cell_width / 2, // Bottom-left corner
      xIndex + cell_width / 2, // Bottom-right corner
      xIndex + cell_width / 2, // Top-right corner
      xIndex - cell_width / 2  // Top-left corner
    };
    std::vector<float> corners_y = {
      yIndex - cell_height / 2, // Bottom-left corner
      yIndex - cell_height / 2, // Bottom-right corner
      yIndex + cell_height / 2, // Top-right corner
      yIndex + cell_height / 2  // Top-left corner
    };

    for (int i = corners_x[0];i<=corners_x[1];i++){
      for (int j = corners_y[0];j<=corners_y[3];j++)
      {
        int obsIndex = get1DIndex(i, j);
        if (obsIndex>=0 && obsIndex < width * height)
            data[obsIndex] = true;
      }
    }

  }

  
};
