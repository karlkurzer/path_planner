//###################################################
//                      (NOT YET HYBRID) A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-12-15
//###################################################

#include <ctime>
#include <cstring>
#include <iostream>
//#include <vector>

#include <ros/ros.h>
//#include <nav_msgs/OccupancyGrid.h>
//#include <nav_msgs/Path.h>
//#include <geometry_msgs/PoseStamped.h>

#include "subscribeandpublish.h"
//#include "dubins.h"
//#include "node3d.h"
//#include "path.h"

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
    message("Approaching Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");
    ros::init(argc, argv, "a_star");
    SubscribeAndPublish supPub;
    ros::spin();
    return 0;
}

////###################################################
////                            TRAVERSABILITY CHECKING
////###################################################

//inline bool traversable(const int x, const int y, const Node3D& node) {
//    int xSucc, ySucc;

//    for (int i = 0; i < dir; i++) {
//        xSucc = x + dx[i];
//        ySucc = y + dy[i];

//        if (xSucc == node.getX() && ySucc == node.getY()) {
//            return false;
//        }
//    }

//    return true;
//}
