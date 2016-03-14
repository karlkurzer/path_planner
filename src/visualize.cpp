#include "visualize.h"

void Visualize::publishNode3D(Node3D& node) {
  visualization_msgs::Marker marker;

  // CREATE THE MARKER
  marker.header.frame_id = "path";
  marker.header.stamp = ros::Time::now();
  marker.ns = "node3Dns";
  marker.action = idNode3D == 0 ? 3 : 0;
  marker.id = idNode3D++;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale.x = Node3D::dx[0];
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 0.5;
  marker.color.r = teal.red;
  marker.color.g = teal.green;
  marker.color.b = teal.blue;
  marker.pose.position.x = node.getX();
  marker.pose.position.y = node.getY();
  marker.pose.position.z = 1;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());

  // PUBLISH THE MARKERARRAY
  pubNode3D.publish(marker);
}
