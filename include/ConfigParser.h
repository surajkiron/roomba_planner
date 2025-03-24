#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <string>
#include <yaml-cpp/yaml.h>
#include "BFSPathPlanner.h"
#include "DijkstraPathPlanner.h"
#include "PathPlannerInterface.h"

struct OccupancyGridConfig {
  int width;
  int height;
  double resolution_m;
  std::vector<Eigen::Vector4f> obstacles;
};

class ConfigParser {
public:
  ConfigParser(const std::string& configFilePath) {
    YAML::Node config = YAML::LoadFile(configFilePath);
    occupancyGrid.width = config["OccupancyGrid"]["width"].as<int>();
    occupancyGrid.height = config["OccupancyGrid"]["height"].as<int>();
    occupancyGrid.resolution_m = config["OccupancyGrid"]["resolution_m"].as<double>();
    for (const auto& obstacle : config["OccupancyGrid"]["obstacles"]) {
      Eigen::Vector4f obstacleVec;
      obstacleVec << obstacle[0].as<float>(), obstacle[1].as<float>(), obstacle[2].as<float>(), obstacle[3].as<float>();
      occupancyGrid.obstacles.push_back(obstacleVec);
    }
    auto Start = config["Start"];
    auto End = config["End"];
    start << Start[0].as<float>(), Start[1].as<float>(), Start[2].as<float>();
    end << End[0].as<float>(), End[1].as<float>(), End[2].as<float>();
    std::string plannerType = config["PlannerType"].as<std::string>();
    if (plannerType == "BFS") {
        planner = Planner::BFS;
    } else if (plannerType == "Dijkstra") {
        planner = Planner::Dijkstra;
    } else {
        std::cerr << "Unknown planner type" << std::endl;
        return;
    }
  }



public:
  OccupancyGridConfig occupancyGrid;
  Planner planner;
  Eigen::Vector3f start;
  Eigen::Vector3f end;
};

#endif // CONFIG_PARSER_H