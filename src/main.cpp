/**
 * @file main.cpp
 * @brief Main entry point of the program, starts an instance of SubscribeAndPublish
 */

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
/**
 * @fn message(const T& msg, T1 val = T1())
 * @brief Convenience method to display text
 */
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
/**
 * @fn main(int argc, char** argv)
 * @brief Starting the program
 * @param argc The standard main argument count
 * @param argv The standard main argument value
 * @return 0
 */
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
