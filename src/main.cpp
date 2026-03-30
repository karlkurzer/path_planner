#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "constants.h"
#include "planner.h"

template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

int main(int argc, char** argv) {
  message<std::string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");
  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("a_star");

  HybridAStar::Planner hy(node);
  hy.plan();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
