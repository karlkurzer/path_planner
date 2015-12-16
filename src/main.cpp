//###################################################
//                      (NOT YET HYBRID) A* ALGORITHM
//	AUTHOR:		Karl Kurzer
//	WRITTEN:	2015-12-15
//###################################################

#include <ctime>
#include <cstring>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

#include "subscribeandpublish.h"
#include "dubins.h"
#include "node3d.h"
#include "path.h"

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

////###################################################
////                                          SCENARIOS
////###################################################

//void createScenario(Node3D& start3D, Node3D& goal3D, Node2D& start2D, Node2D& goal2D, int scenario) {
//    int xStart, yStart, xGoal, yGoal;

//    // create empty grid
//    for (int i = 0; i < length; i++) {
//        for (int j = 0; j < width; j++) {
//            grid[i][j] = true;
//            path[i][j] = false;
//            listOpen2D[i][j] = false;
//            listClosed2D[i][j] = false;
//            listCost2D[i][j] = 0;
//            listCostToGo2D[i][j] = 0;
//            listCostGoal2D[i][j] = 0;
//            total2D = 0;

//            for (int k = 0; k < dir; k++) {
//                listOpen3D[i][j][k] = false;
//                listClosed3D[i][j][k] = false;
//                listCost3D[i][j][k] = 0;
//            }
//        }
//    }

//    switch (scenario) {
//    //
//    case 0:
//        start3D.setX(5);
//        start3D.setY(width - 1);
//        goal3D.setX(5);
//        goal3D.setY(width / 2);
//        goal3D.setT(0);
//        goal2D.setX(5);
//        goal2D.setY(width / 2);
//        break;

//    case 1:
//        start3D.setX(length - 4);
//        start3D.setY(width / 2);
//        goal3D.setX(5);
//        goal3D.setY(width / 2);
//        goal3D.setT(0);
//        goal2D.setX(5);
//        goal2D.setY(width / 2);

//        // horizontal
//        for (int j = width / 4; j < width * 3 / 4; j++)
//        { grid[length / 2][j] = false; }

//        // vertical
//        for (int j = length / 2; j < length * 3 / 4; j++)
//        { grid[j][width / 4] = false; }

//        for (int k = length / 2; k < length * 3 / 4; k++)
//        { grid[k][width * 3 / 4] = false; }

//        break;

//    // straigth
//    case 4:
//        start3D.setX(length - 4);
//        start3D.setY(width / 2);
//        goal3D.setX(5);
//        goal3D.setY(width / 2);
//        goal3D.setT(0);
//        goal2D.setX(5);
//        goal2D.setY(width / 2);
//        break;

//    // straigth
//    case 5:
//        start3D.setX(length / 2);
//        start3D.setY(width * 3 / 4);
//        start3D.setT(270);
//        goal3D.setX(length / 2);
//        goal3D.setY(width / 4);
//        goal3D.setT(270);
//        goal2D.setX(length / 2);
//        goal2D.setY(width / 4);
//        break;

//    case 2:
//        start3D.setX(2);
//        start3D.setY(2);
//        goal3D.setX(5);
//        goal3D.setY(56);
//        goal2D.setX(5);
//        goal2D.setY(56);

//        if (width == 60 && length == 30) {
//            for (int i = 0; i < 7; ++i)
//            { grid[i][7] = false; }

//            for (int i = 12; i < 18; ++i)
//            { grid[4][i] = false; }

//            for (int i = 4; i < 13; ++i)
//            { grid[i][18] = false; }

//            for (int i = 0; i < 22; ++i)
//            { grid[i][34] = false; }

//            for (int i = 4; i < 11; ++i)
//            { grid[i][51] = false; }

//            for (int i = 51; i < 59; ++i)
//            { grid[11][i] = false; }
//        } else { message("Expected the width to be 60 and the length to be 30"); }

//        break;

//    //random szenario
//    default:
//        // start node
//        xStart = rand() % (length - 1);
//        yStart = rand() % (width - 1);
//        start3D.setX(xStart);
//        start3D.setY(yStart);
//        // goal node
//        xGoal = rand() % (length - 1);
//        yGoal = rand() % (width - 1);

//        while (abs(xStart - xGoal) < length / 3) {
//            xGoal = rand() % (length - 1);
//        }

//        goal3D.setX(xGoal);
//        goal3D.setY(yGoal);
//        goal2D.setX(xGoal);
//        goal2D.setY(yGoal);

//        // create obstacles on the grid
//        for (int i = 0; i < length; i++) {
//            for (int j = 0; j < width; j++) {
//                int random = rand() % 100;

//                // create random obstacles, obstacles are more likely to appear together
//                if (random < 1 && traversable(i, j, start3D) && traversable(i, j, goal3D))
//                { grid[i][j] = false; }
//                else if (random < 80 && j < width && j != 0 && grid[i][j - 1] == false && traversable(i, j, start3D) && traversable(i, j, goal3D))
//                { grid[i][j] = false; }
//            }
//        }

//        break;
//    }

//    message("created grid"); cout << length << " by " << width << endl;
//    cout << "The heading for the start state is " << start2D.getH() << " degrees north" << endl;
//    cout << "The heading for the goal state is " << goal2D.getH() << " degrees north" << endl;
//    cout << "The turning radius of the vehicle is " << 1.5 << " cells" << endl;
//}
