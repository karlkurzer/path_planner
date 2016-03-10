#include "path.h"

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::tracePath(Node3D* node, int count) {
  if (node == nullptr) { return; }

  addSegment(node);
  addNode(node, count);
  count++;
  addVehicle(node, count);
  count++;
  tracePath(node->getPred(), count);
}

// ___________
// ADD SEGMENT
void Path::addSegment(Node3D* node) {
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
void Path::addNode(Node3D* node, int count) {
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
  pathNode.color.r = 0.5;
  pathNode.color.g = 0.0;
  pathNode.color.b = 0.5;
  pathNode.pose.position.x = node->getX();
  pathNode.pose.position.y = node->getY();
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(Node3D* node, int count) {
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
  pathVehicle.color.r = 0.5;
  pathVehicle.color.g = 0.0;
  pathVehicle.color.b = 0.5;
  pathVehicle.pose.position.x = node->getX();
  pathVehicle.pose.position.y = node->getY();
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node->getT());
  pathVehicles.markers.push_back(pathVehicle);
}

//###################################################
//                                     TRACE 3D NODES
//###################################################
geometry_msgs::PoseArray Path::getNodes3D(int width, int height, int depth, int length,
    bool* closed) {
  geometry_msgs::PoseArray nodes;
  nodes.header.frame_id = "path";
  nodes.header.stamp = ros::Time::now();
  int count = 0;

  for (int i = 0; i < length; ++i) {
    if (closed[i]) {
      count++;
      geometry_msgs::Pose node;
      // center in cell +0.5
      node.position.x = i % width + 0.5;
      node.position.y = (i / width) % height + 0.5;
      node.orientation = tf::createQuaternionMsgFromYaw((i / (width * height) % depth) * constants::deltaHeadingRad);

      nodes.poses.push_back(node);
    }
  }

  std::cout << count << " 3D nodes expanded" << std::endl;
  return nodes;
}

//###################################################
//                                     TRACE 2D NODES
//###################################################
visualization_msgs::MarkerArray Path::getNodes2D(int width, int height, float* cost2d) {
  visualization_msgs::MarkerArray nodes;

  int count = 0;

  for (int i = 0; i < width * height; ++i) {
    if (cost2d[i] != 0) {
      visualization_msgs::Marker node;

      // delete all previous markers
      if (count == 0) {
        node.action = 3;
      }

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

  std::cout << count << " 2D nodes expanded" << std::endl;
  return nodes;
}

//###################################################
//                                       COST HEATMAP
//###################################################
sensor_msgs::PointCloud2 Path::getCosts(int width, int height, int depth, float* cost, float* costToGo) {
  sensor_msgs::PointCloud2 costCloud;
  costCloud.header.frame_id = "map";
  costCloud.header.stamp = ros::Time::now();
  costCloud.height = height;
  costCloud.width = width;
  float sum;
  float min;
  int idx;

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      sum = 0;
      min = 1000;

      // iterate over all headings
      for (int k = 0; k < depth; ++k) {
        //        sum += cost[k * width * height + j * width + i];
        idx = k * width * height + j * width + i;
        min = idx;
        std::cout<< idx <<"\n";
        if (cost[idx] > 0 && cost[idx]+costToGo[idx] < min) {
//          min = cost[idx]+costToGo[idx];
        }
      }

      //        sum /= count;
      costCloud.data.push_back();
    }
  }

  return costGrid;
}
