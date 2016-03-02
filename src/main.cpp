//###################################################
//                      HYBRID A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "subscribeandpublish.h"

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

  message("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  ros::init(argc, argv, "a_star");

  SubscribeAndPublish supPub;

  ros::spin();
  return 0;
}
