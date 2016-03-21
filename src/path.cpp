#include "path.h"

//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
    Node3D* node = new Node3D();
    path.poses.clear();
    pathNodes.markers.clear();
    pathVehicles.markers.clear();
    addNode(node, 0);
    addVehicle(node, 1);
    publishPath();
    publishPathNodes();
    publishPathVehicles();
    delete node;
}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::tracePath(const Node3D* node, int i) {
  if (i == 0) {
    path.header.stamp = ros::Time::now();
  }

  if (node == nullptr) { return; }

  addSegment(node);
  addNode(node, i);
  i++;
  addVehicle(node, i);
  i++;

  if (node->getPred() == node) {
    std::cout << "loop";
    return;
  }

  tracePath(node->getPred(), i);
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
void Path::addNode(const Node3D* node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
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

void Path::addVehicle(const Node3D* node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markers
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
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

////###################################################
////                                     TRACE 3D NODES
////###################################################
//geometry_msgs::PoseArray Path::getNodes3D(int width, int height, int depth, int length, Node3D* closed) {
//  geometry_msgs::PoseArray nodes;
//  nodes.header.frame_id = "path";
//  nodes.header.stamp = ros::Time::now();
//  int count = 0;

//  for (int i = 0; i < length; ++i) {
//    if (closed[i].isClosed()) {
//      count++;
//      geometry_msgs::Pose node;
//      // center in cell +0.5
//      node.position.x = i % width + 0.5;
//      node.position.y = (i / width) % height + 0.5;
//      node.orientation = tf::createQuaternionMsgFromYaw((i / (width * height) % depth) * constants::deltaHeadingRad);

//      nodes.poses.push_back(node);
//    }
//  }

//  if (constants::coutDEBUG) {
//    std::cout << count << " 3D nodes expanded" << std::endl;
//  }

//  return nodes;
//}

////###################################################
////                                     TRACE 2D NODES
////###################################################
//visualization_msgs::MarkerArray Path::getNodes2D(int width, int height, float* cost2d) {
//  visualization_msgs::MarkerArray nodes;
//  visualization_msgs::Marker node;

//  int count = 0;

//  for (int i = 0; i < width * height; ++i) {
//    if (cost2d[i] != 0) {

//      // delete all previous markers
//      node.action = count == 0 ? 3 : 0;
//      node.header.frame_id = "path";
//      node.header.stamp = ros::Time::now();
//      node.id = count;
//      node.type = visualization_msgs::Marker::SPHERE;
//      node.scale.x = 0.1;
//      node.scale.y = 0.1;
//      node.scale.z = 0.1;
//      node.color.a = 1.0;
//      node.color.r = 1.0;
//      node.color.g = 0.0;
//      node.color.b = 0.0;
//      // center in cell +0.5
//      node.pose.position.x = i % width + 0.5;
//      node.pose.position.y = (i / width) % height + 0.5;
//      nodes.markers.push_back(node);
//      count++;
//    }
//  }

//  if (constants::coutDEBUG) {
//    std::cout << count << " 2D nodes expanded" << std::endl;
//  }

//  return nodes;
//}
