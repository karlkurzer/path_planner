#include "path.h"

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::tracePath(const Node3D* node, int count) {
  if (node == nullptr) { return; }

  addSegment(node);
  addNode(node, count);
  count++;
  addVehicle(node, count);
  count++;
  if (node->getPred() == node){
      std::cout <<"loop";
      return;
  }
  tracePath(node->getPred(), count);
}

// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D* node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node->getX();
  vertex.pose.position.y = node->getY();
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE
void Path::addNode(const Node3D* node, int count) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (count == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time::now();
  pathNode.id = count;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;
  pathNode.color.r = purple.red;
  pathNode.color.g = purple.green;
  pathNode.color.b = purple.blue;
  pathNode.pose.position.x = node->getX();
  pathNode.pose.position.y = node->getY();
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D* node, int count) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markers
  if (count == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time::now();
  pathVehicle.id = count;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = constants::length - constants::bloating * 2;
  pathVehicle.scale.y = constants::width - constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;
  pathVehicle.color.r = teal.red;
  pathVehicle.color.g = teal.green;
  pathVehicle.color.b = teal.blue;
  pathVehicle.pose.position.x = node->getX();
  pathVehicle.pose.position.y = node->getY();
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node->getT());
  pathVehicles.markers.push_back(pathVehicle);
}

//###################################################
//                                     TRACE 3D NODES
//###################################################
geometry_msgs::PoseArray Path::getNodes3D(int width, int height, int depth, int length, Node3D* closed) {
  geometry_msgs::PoseArray nodes;
  nodes.header.frame_id = "path";
  nodes.header.stamp = ros::Time::now();
  int count = 0;

  for (int i = 0; i < length; ++i) {
    if (closed[i].isClosed()) {
      count++;
      geometry_msgs::Pose node;
      // center in cell +0.5
      node.position.x = i % width + 0.5;
      node.position.y = (i / width) % height + 0.5;
      node.orientation = tf::createQuaternionMsgFromYaw((i / (width * height) % depth) * constants::deltaHeadingRad);

      nodes.poses.push_back(node);
    }
  }

  if (constants::coutDEBUG) {
    std::cout << count << " 3D nodes expanded" << std::endl;
  }

  return nodes;
}

//###################################################
//                                     TRACE 2D NODES
//###################################################
visualization_msgs::MarkerArray Path::getNodes2D(int width, int height, float* cost2d) {
  visualization_msgs::MarkerArray nodes;
  visualization_msgs::Marker node;

  int count = 0;

  for (int i = 0; i < width * height; ++i) {
    if (cost2d[i] != 0) {

      // delete all previous markers
      node.action = count == 0 ? 3 : 0;
      node.header.frame_id = "path";
      node.header.stamp = ros::Time::now();
      node.id = count;
      node.type = visualization_msgs::Marker::SPHERE;
      node.scale.x = 0.1;
      node.scale.y = 0.1;
      node.scale.z = 0.1;
      node.color.a = 1.0;
      node.color.r = 1.0;
      node.color.g = 0.0;
      node.color.b = 0.0;
      // center in cell +0.5
      node.pose.position.x = i % width + 0.5;
      node.pose.position.y = (i / width) % height + 0.5;
      nodes.markers.push_back(node);
      count++;
    }
  }

  if (constants::coutDEBUG) {
    std::cout << count << " 2D nodes expanded" << std::endl;
  }

  return nodes;
}

//###################################################
//                                       COST HEATMAP
//###################################################
visualization_msgs::MarkerArray Path::getCosts(int width, int height, int depth, float* cost, float* costToGo) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  int idx;
  bool cube;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // ________________________________
  // DETERMINE THE MAX AND MIN VALUES
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // iterate over all headings
    for (int k = 0; k < depth; ++k) {
      idx = k * width * height + i;

      // set the minimum for the cell
      if (cost[idx] > 0 && cost[idx] + costToGo[idx] < values[i]) {
        values[i] = cost[idx] + costToGo[idx];
      }
    }

    // set a new minimum
    if (values[i] > 0 && values[i] < min) {
      min = values[i];
    }

    // set a new maximum
    if (values[i] > 0 && values[i] > max && values[i] != 1000) {
      max = values[i];
    }
  }

  // _______________
  // PAINT THE CUBES
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (values[i] != 1000) {
      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }


      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = 1.0;
      costCube.scale.y = 1.0;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = i % width + 0.5;
      costCube.pose.position.y = (i / width) % height + 0.5;
      costCubes.markers.push_back(costCube);
      cube = false;
    }
  }

  if (constants::coutDEBUG) {
    std::cout << "min cost: " << min << " | max cost: " << max << std::endl;
  }

  return costCubes;
}
