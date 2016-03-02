#ifndef SUBSCRIBEANDPUBLISH
#define SUBSCRIBEANDPUBLISH

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "node3d.h"
#include "path.h"
#include "constants.h"

class SubscribeAndPublish {
 public:
  //###################################################
  //                                        CONSTRUCTOR
  //###################################################
  SubscribeAndPublish() {
    // topics to PUBLISH
    //    pub_rcv = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    pub_path = n.advertise<nav_msgs::Path>("/path", 1);
    pub_pathNodes = n.advertise<visualization_msgs::MarkerArray>("/pathNodes", 1);
    pub_nodes3D = n.advertise<geometry_msgs::PoseArray>("/nodes3D", 1);
    pub_nodes2D = n.advertise<visualization_msgs::MarkerArray>("/nodes2D", 1);
    pub_costMap = n.advertise<nav_msgs::OccupancyGrid>("/costMap", 1);
    pub_start = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // topics to SUBSCRIBE
    if (manual) {
      sub_map = n.subscribe("/map", 1, &SubscribeAndPublish::setMap, this);
    } else {
      sub_map = n.subscribe("/occ_map", 1, &SubscribeAndPublish::setMap, this);
    }

    sub_goal = n.subscribe("/move_base_simple/goal", 1, &SubscribeAndPublish::setGoal, this);
    sub_start = n.subscribe("/initialpose", 1, &SubscribeAndPublish::setStart, this);
  }

  //###################################################
  //                                                MAP
  //###################################################
  void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
    std::cout << "I am seeing the map..." << std::endl;
    grid = map;

    bloatObstacles(grid);

    // plan if the switch is not set to manual
    if (!manual) { plan();}
  }

  //###################################################
  //                                      PLAN THE PATH
  //###################################################
  void plan() {
    // if a start as well as goal are defined go ahead and plan
    if (start != nullptr && goal != nullptr) {
      // LISTS dynamically allocated ROW MAJOR ORDER
      int width = grid->info.width;
      int height = grid->info.height;
      int depth = constants::orientations;
      int length = width * height * depth;
      // define list pointers
      bool* open;
      bool* closed;
      float* cost;
      float* costToGo;
      float* cost2d;

      // initialize all lists
      open = new bool [length]();
      closed = new bool [length]();
      cost = new float [length]();
      costToGo = new float [length]();
      // 2D COSTS
      cost2d = new float [width * height]();

      // retrieving goal position
      float x = goal->pose.position.x;
      float y = goal->pose.position.y;
      float t = tf::getYaw(goal->pose.orientation) * 180 / M_PI - 90;

      if (t < 0) {
        t = 360 + t;
      }

      if (t >= 360) {
        t -= 360;
      }

      Node3D nGoal(x, y, t, 0, 0, nullptr);

      // retrieving start position
      x = start->pose.pose.position.x;
      y = start->pose.pose.position.y;
      t = tf::getYaw(start->pose.pose.orientation) * 180 / M_PI - 90;

      if (t < 0) {
        t = 360 + t;
      }

      if (t >= 360) {
        t -= 360;
      }

      Node3D nStart(x, y, t, 0, 0, nullptr);

      ros::Time t0 = ros::Time::now();
      Path path(Node3D::aStar(nStart, nGoal, grid, width, height, depth, length, open, closed, cost,
                              costToGo, cost2d), "path");
      ros::Time t1 = ros::Time::now();
      ros::Duration d(t1 - t0);
      std::cout << "Time in ms: " << d * 1000 << std::endl;
      //      std::cout << "Length in m: " << path.getLength() << std::endl;
      // publish the results of the search
      pub_path.publish(path.getPath());
      pub_pathNodes.publish(path.getPathNodes());
      pub_nodes3D.publish(Path::getNodes3D(width, height, depth, length, closed));
      pub_nodes2D.publish(Path::getNodes2D(width, height, cost2d));
      pub_costMap.publish(Path::getCosts(width, height, depth, cost));

      // LISTS deleted
      delete [] open;
      delete [] closed;
      delete [] cost;
      delete [] costToGo;
      delete [] cost2d;
    } else {
      std::cout << "missing goal or start" << std::endl;
    }
  }

  //###################################################
  //                                    INITIALIZE GOAL
  //###################################################
  void setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
    // retrieving goal position
    float x = end->pose.position.x;
    float y = end->pose.position.y;
    float t = tf::getYaw(end->pose.orientation) * 180 / M_PI - 90;

    if (t < 0) {
      t = 360 + t;
    }

    if (t >= 360) {
      t -= 360;
    }

    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << t << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      goal = end;

      if (manual) { plan();}

    } else {
      std::cout << "invalid goal x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }

  //###################################################
  //                                   INITIALIZE START
  //###################################################
  void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    float x = initial->pose.pose.position.x;
    float y = initial->pose.pose.position.y;
    float t = tf::getYaw(initial->pose.pose.orientation) * 180 / M_PI - 90;
    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();


    if (t < 0) {
      t = 360 + t;
    }

    if (t >= 360) {
      t -= 360;
    }

    std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << t << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
      start = initial;

      if (manual) { plan();}

      pub_start.publish(startN);
    } else {
      std::cout << "invalid start x:" << x << " y:" << y << " t:" << t << std::endl;
    }
  }

  //###################################################
  //                                    COLLISIONLOOKUP
  //###################################################

  // ____________________
  // IMPORTANT PARAMETERS
  const double bloating = 0;
  const double width = 1.75;
  const double length = 2.65;
  const int orientations = 72;
  // cell size
  const float cSize = 1;
  // bounding box size length/width
  const int size = std::ceil((sqrt(width * width + length* length) + 2 * bloating) / cSize) + 4;
  bool DEBUG = false;

  // LOOKUP STRUCTS
  struct relPos {
    int x;
    int y;
  };

  struct config {
    int theta;
    int length;
    relPos pos[32];
  };

  // SIGN FUNCTION
  int sign(double x) {
    if (x >= 0) { return 1; }

    if (x < 0) { return -1; }
  }

  // LOOKUP CREATION
  constants::config collisionLookup() {


    struct point {
      double x;
      double y;
    };

    // ______________________
    // VARIABLES FOR ROTATION
    //center of the rectangle
    point c;
    point temp;
    // points of the rectangle
    point p[4];
    point nP[4];

    // turning angles and increments
    double dTheta = M_PI / orientations * 2;
    double theta = 0;

    // set points of rectangle
    p[0].x = (double)size / 2 - width / 2 / cSize;
    p[0].y = (double)size / 2 - length / 2 / cSize;

    p[1].x = (double)size / 2 - width / 2 / cSize;
    p[1].y = (double)size / 2 + length / 2 / cSize;

    p[2].x = (double)size / 2 + width / 2 / cSize;
    p[2].y = (double)size / 2 + length / 2 / cSize;

    p[3].x = (double)size / 2 + width / 2 / cSize;
    p[3].y = (double)size / 2 - length / 2 / cSize;
    c.x = (double)size / 2;
    c.y = (double) size / 2;

    // ____________________________
    // VARIABLES FOR GRID TRAVERSAL
    // vector for grid traversal
    point t;
    point start;
    point end;
    // cell index
    int X;
    int Y;
    // t value for crossing vertical and horizontal boundary
    double tMaxX;
    double tMaxY;
    // t value for width/heigth of cell
    double tDeltaX;
    double tDeltaY;
    // positive or negative step direction
    int stepX;
    int stepY;
    // grid
    bool cSpace[size * size];
    bool inside = false;
    int hcross1;
    int hcross2;

    // _____________________________
    // VARIABLES FOR LOOKUP CREATION
    int count = 0;
    constants::config lookup[orientations];

    for (int o = 0; o < orientations; ++o) {
      if (DEBUG) { std::cout << "\ndegrees: " << theta * 180 / M_PI << std::endl; }

      // initialize cSpace
      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          cSpace[i * size + j] = false;
        }
      }

      // shape rotation
      for (int j = 0; j < 4; ++j) {
        // translate point to origin
        temp.x = p[j].x - c.x;
        temp.y = p[j].y - c.y;

        // rotate and shift back
        nP[j].x = temp.x * cos(theta) - temp.y * sin(theta) + c.x;
        nP[j].y = temp.x * sin(theta) + temp.y * cos(theta) + c.y;
        //      std::cout << j << "--> pX: " << nP[j].x << " pY: " << nP[j].y << std::endl;
      }

      // create the next angle
      theta += dTheta;

      // cell traversal clockwise
      for (int k = 0; k < 4; ++k) {
        // create the vectors clockwise
        if (k < 3) {
          start = nP[k];
          end = nP[k + 1];
        } else {
          start = nP[k];
          end = nP[0];
        }

        //set indexes
        X = trunc(start.x);
        Y = trunc(start.y);
        //      std::cout << "StartCell: " << X << "," << Y << std::endl;
        cSpace[Y * size + X] = true;
        t.x = end.x - start.x;
        t.y = end.y - start.y;
        stepX = sign(t.x);
        stepY = sign(t.y);

        // width and height normalized by t
        if (t.x != 0) {
          tDeltaX = cSize / std::abs(t.x);
        } else {
          tDeltaX = 1000;
        }

        if (t.y != 0) {
          tDeltaY = cSize / std::abs(t.y);
        } else {
          tDeltaY = 1000;
        }

        // set maximum traversal values
        if (stepX > 0) {
          tMaxX = tDeltaX * (1 - (start.x / cSize - (long)(start.x / cSize)));
        } else {
          tMaxX = tDeltaX * (1 - (1 - (start.x / cSize - (long)(start.x / cSize))));
        }

        if (stepY > 0) {
          tMaxY = tDeltaY * (1 - (start.y / cSize - (long)(start.y / cSize)));
        } else {
          tMaxY = tDeltaY * (1 - (1 - (start.y / cSize - (long)(start.y / cSize))));
        }

        while (trunc(end.x) != X || trunc(end.y) != Y) {
          // only increment x if the t length is smaller and the result will be closer to the goal
          if (tMaxX < tMaxY && std::abs(X + stepX - trunc(end.x)) < std::abs(X - trunc(end.x))) {
            tMaxX = tMaxX + tDeltaX;
            X = X + stepX;
            cSpace[Y * size + X] = true;
            //          std::cout << "Cell: " << X << "," << Y << std::endl;
            // only increment y if the t length is smaller and the result will be closer to the goal
          } else if (tMaxY < tMaxX && std::abs(Y + stepY - trunc(end.y)) < std::abs(Y - trunc(end.y))) {
            tMaxY = tMaxY + tDeltaY;
            Y = Y + stepY;
            cSpace[Y * size + X] = true;
            //          std::cout << "Cell: " << X << "," << Y << std::endl;
          } else if (2 >= std::abs(X - trunc(end.x)) + std::abs(Y - trunc(end.y))) {
            if (std::abs(X - trunc(end.x)) > std::abs(Y - trunc(end.y))) {
              X = X + stepX;
              cSpace[Y * size + X] = true;
            } else {
              Y = Y + stepY;
              cSpace[Y * size + X] = true;
            }
          } else {
            // this SHOULD NOT happen
            std::cout << "\n--->tie occured, please check for error in script\n";
            break;
          }
        }
      }

      // FILL THE SHAPE
      for (int i = 0; i < size; ++i) {
        // set inside to false
        inside = false;

        for (int j = 0; j < size; ++j) {

          // determine horizontal crossings
          for (int k = 0; k < size; ++k) {
            if (cSpace[i * size + k] && !inside) {
              hcross1 = k;
              inside = true;
            }

            if (cSpace[i * size + k] && inside) {
              hcross2 = k;
            }
          }

          // if inside fill
          if (j > hcross1 && j < hcross2 && inside) {
            cSpace[i * size + j] = true;
          }
        }
      }

      // GENERATE THE ACTUAL LOOKUP
      count = 0;

      for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
          if (cSpace[i * size + j]) {
            // compute the relative position of the car cells
            lookup[o].pos[count].x = j - trunc(c.x);
            lookup[o].pos[count].y = i - trunc(c.x);
            // add one for the length of the current list
            count++;
          }
        }
      }

      lookup[o].length = count;

      if (DEBUG) {
        //DEBUG
        for (int i = 0; i < size; ++i) {
          std::cout << "\n";

          for (int j = 0; j < size; ++j) {
            if (cSpace[i * size + j]) {
              std::cout << "#";
            } else {
              std::cout << ".";
            }
          }
        }

        //TESTING
        std::cout << "\n\nthe center is at " << c.x << " | " << c.y << std::endl;

        for (int i = 0; i < lookup[o].length; ++i) {
          std::cout << "[" << i << "]\t" << lookup[o].pos[i].x << " | " << lookup[o].pos[i].y << std::endl;
        }
      }
    }

    return *lookup;
  }

  //###################################################
  //                                   OBSTACLEBLOATING
  //###################################################
  void bloatObstacles(nav_msgs::OccupancyGrid::Ptr& grid) {
    int dx[] = { 1,  1,  0,  -1,  -1, -1,   0,    1 };
    int dy[] = { 0,  1,  1,   1,   0, -1,  -1,   -1 };
    int height = grid->info.height;
    int width = grid->info.width;
    int length = height * width;
    int x, y, xSucc, ySucc;
    bool* bloating;
    bloating = new bool[length]();

    for (int i = 0; i < length; ++i) {
      x = i % width;
      y = (i / width) % height;

      if (grid->data[i]) {
        // bloat the obstacle
        for (int j = 0; j < 8; ++j) {
          xSucc = x + dx[j];
          ySucc = y + dy[j];

          if (xSucc >= 0 && xSucc < width && ySucc >= 0 && ySucc < height) {
            bloating[ySucc * width + xSucc] = true;
          }
        }
      }
    }

    for (int i = 0; i < length; ++i) {
      grid->data[i] = bloating[i];
    }

    delete [] bloating;
  }

 private:
  bool manual = true;
  ros::NodeHandle n;
  //  ros::Publisher pub_rcv;
  ros::Publisher pub_path;
  ros::Publisher pub_pathNodes;
  ros::Publisher pub_nodes3D;
  ros::Publisher pub_nodes2D;
  ros::Publisher pub_costMap;
  ros::Publisher pub_start;
  ros::Subscriber sub_map;
  ros::Subscriber sub_goal;
  ros::Subscriber sub_start;
  nav_msgs::OccupancyGrid::Ptr grid;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr start;
  geometry_msgs::PoseStamped::ConstPtr goal;
};

#endif // SUBSCRIBEANDPUBLISH



