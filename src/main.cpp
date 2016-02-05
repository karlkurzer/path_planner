//###################################################
//                      HYBRID A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-02-05
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "subscribeandpublish.h"


//###################################################
//                                                CFG
//###################################################
struct cfg {
  bool penalty;
  bool dubins;
  bool twoD;
};

//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
void message(const std::string& msg) {
  std::cout << "\n### " << msg << std::endl;
}

//###################################################
//                                               MAIN
//###################################################
int main(int argc, char** argv) {
  cfg config;
  config.penalty = false;
  config.dubins = false;
  config.twoD = false;
  message("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");
  ros::init(argc, argv, "a_star");
  SubscribeAndPublish supPub;
  ros::spin();
  return 0;
}
