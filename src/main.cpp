//###################################################
//                      HYBRID A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "subscribeandpublish.h"
#include "constants.h"

//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
int main(int argc, char** argv) {

  message<string,int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  message("cell size: ", constants::cellSize);

  if (constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, "a_star");

  SubscribeAndPublish supPub;

  ros::spin();
  return 0;
}
