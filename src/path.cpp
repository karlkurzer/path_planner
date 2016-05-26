#include "path.h"

using namespace HybridAStar;


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
  vertex.pose.position.x = node->getX()*Constants::cellSize;
  vertex.pose.position.y = node->getY()*Constants::cellSize;
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
  pathNode.color.r = Constants::purple.red;
  pathNode.color.g = Constants::purple.green;
  pathNode.color.b = Constants::purple.blue;
  pathNode.pose.position.x = node->getX()*Constants::cellSize;
  pathNode.pose.position.y = node->getY()*Constants::cellSize;
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
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;
  pathVehicle.color.r = Constants::teal.red;
  pathVehicle.color.g = Constants::teal.green;
  pathVehicle.color.b = Constants::teal.blue;
  pathVehicle.pose.position.x = node->getX()*Constants::cellSize;
  pathVehicle.pose.position.y = node->getY()*Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node->getT());
  pathVehicles.markers.push_back(pathVehicle);
}
