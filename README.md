# Planning and Controls Assessment

## Problem Statement

A circular roomba robot is present in an already mapped environment. The map is in the format of a binary occupancy
grid.
<p align="center"><img src="assets/grid-example.png" width="50%"></p>

Each cell in the grid contains a boolean value indicating if an obstacle is present at a given location, i.e `true -> obstacle` and `false -> free`. The `OccupancyGrid` struct contains a 1d array of boolean
that denotes a 2d grid in a row major format. The intent is to find a safe **collision-free** path from a given location
to the vicinity of the goal location.

Create a class that inherits the `PathPlannerInterface` class, and implement the `getCollisionFreePath()` pure virtual
function to provide a trajectory which can be followed by the robot. Note that the start position and goal position may
be any point within the occupancy grid. Take any assumptions based on the code provided already in the repository, but
state the assumptions clearly in the Assumptions section below. We will analyze the design of the system as well as the
algorithm. Try to stick to Eigen and STL libs for implementation.

## System Description

### Grid

The origin of the occupancy grid is the bottom left of the image. The positive x-axis is to the right and positive
y-axis is upwards. The grid is of size `320 x 320` with a resolution of `0.05 m`. The `OccupancyGrid` class has methods
to set occupancy for a given coordinate, which could be useful for testing solutions. Feel free to edit this part of the
code as well as use any kind of visualization assist (Not needed for this assessment).

### Robot

The robot is a spherical robot of diameter `0.6m` and it operates with a 2d coordinate system with $(x.y. \theta)$ where
$\theta$ is the yaw(orientation) of the robot.

## Dependencies

### Eigen

Eigen is a linear algebra library. Documentation to use Eigen is present
[here](https://eigen.tuxfamily.org/dox/group__QuickRefPage.html). A cheat sheet with quick usages for Eigen Data
structures and functions can be found [here](https://gist.github.com/gocarlos/c91237b02c120c6319612e42fa196d77)

To install Eigen, run the following command:

    sudo apt install libeigen3-dev

### Yaml-cpp
Yaml-cpp library is used to parse config.yml.
```
sudo apt install libyaml-cpp-dev
```
## Build

To build , run

    cmake -B build && cmake --build build && ./bin/CheckPath

Feel free to edit the [main.cpp](src/main.cpp) to run the Path planner with any start and stop locations in any random
grid.

## Assumptions
