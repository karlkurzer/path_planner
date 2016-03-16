#include "visualize.h"

//###################################################
//                                CLEAR VISUALIZATION
//###################################################

void Visualize::clear(){
    poses3D.poses.clear();

    visualization_msgs::MarkerArray costCubes;
    visualization_msgs::Marker costCube;
    // CLEAR THE COST HEATMAP
    costCube.header.frame_id = "path";
    costCube.header.stamp = ros::Time::now();
    costCube.id = 0;
    costCube.action = 3;
    costCubes.markers.push_back(costCube);
    pubNodes3DCosts.publish(costCubes);
}

//###################################################
//                                       CURRENT NODE
//###################################################
void Visualize::publishNode3DPose(Node3D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX();
  pose.pose.position.y = node.getY();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());

  // PUBLISH THE POSE
  pubNode3D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 3D NODES
//###################################################
void Visualize::publishNode3DPoses(Node3D& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX();
  pose.position.y = node.getY();
  pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());

  poses3D.poses.push_back(pose);
  poses3D.header.stamp = ros::Time::now();
  // PUBLISH THE POSEARRAY
  pubNodes3D.publish(poses3D);
}

//###################################################
//                                       COST HEATMAP
//###################################################
void Visualize::publishNode3DCosts(Node3D* nodes, int width, int height, int depth) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  int idx;
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
      if (nodes[idx].getG() > 0 && nodes[idx].getC() < values[i]) {
        values[i] = nodes[idx].getC();
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
    }
  }

  if (constants::coutDEBUG) {
    std::cout << "min cost: " << min << " | max cost: " << max << std::endl;
  }

  // PUBLISH THE COSTCUBES
  pubNodes3DCosts.publish(costCubes);
}
