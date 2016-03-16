//###################################################
//                        TF MODULE FOR THE HYBRID A*
//###################################################
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>

// map pointer
nav_msgs::OccupancyGridPtr grid;

// map callback
void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  std::cout << "Creating transform for map..." << std::endl;
  grid = map;
}

int main(int argc, char** argv) {
  // initiate the broadcaster
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;

  // subscribe to map updates
  ros::Subscriber sub_map = n.subscribe("/occ_map", 1, setMap);
  tf::Pose tfPose;


  ros::Rate r(100);
  tf::TransformBroadcaster broadcaster;

  while (ros::ok()) {
    // transform from geometry msg to TF
    if (grid != nullptr) {
      tf::poseMsgToTF(grid->info.origin, tfPose);
    }

    // odom to map
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tfPose.getOrigin()),
        ros::Time::now(), "odom", "map"));

    // map to path
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(), "map", "path"));
    ros::spinOnce();
    r.sleep();
  }
}
