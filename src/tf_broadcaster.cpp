//###################################################
//                        TF MODULE FOR THE HYBRID A*
//###################################################
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFBroadcaster : public rclcpp::Node {
public:
  TFBroadcaster() : Node("tf_broadcaster") {
    // subscribe to map updates
    sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/occ_map", 1, std::bind(&TFBroadcaster::setMap, this, std::placeholders::_1));

    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TFBroadcaster::broadcast_timer_callback, this));
  }

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose tfPose_;

  void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    RCLCPP_INFO(this->get_logger(), "Creating transform for map...");
    grid_ = map;
  }

  void broadcast_timer_callback() {
    if (grid_ != nullptr) {
      tfPose_ = grid_->info.origin;
    }

    rclcpp::Time now = this->now();

    // odom to map
    geometry_msgs::msg::TransformStamped t_odom_map;
    t_odom_map.header.stamp = now;
    t_odom_map.header.frame_id = "odom";
    t_odom_map.child_frame_id = "map";
    t_odom_map.transform.translation.x = tfPose_.position.x;
    t_odom_map.transform.translation.y = tfPose_.position.y;
    t_odom_map.transform.translation.z = tfPose_.position.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t_odom_map.transform.rotation.x = q.x();
    t_odom_map.transform.rotation.y = q.y();
    t_odom_map.transform.rotation.z = q.z();
    t_odom_map.transform.rotation.w = q.w();
    broadcaster_->sendTransform(t_odom_map);

    // map to path
    geometry_msgs::msg::TransformStamped t_map_path;
    t_map_path.header.stamp = now;
    t_map_path.header.frame_id = "map";
    t_map_path.child_frame_id = "path";
    t_map_path.transform.translation.x = 0;
    t_map_path.transform.translation.y = 0;
    t_map_path.transform.translation.z = 0;
    t_map_path.transform.rotation.x = q.x();
    t_map_path.transform.rotation.y = q.y();
    t_map_path.transform.rotation.z = q.z();
    t_map_path.transform.rotation.w = q.w();
    broadcaster_->sendTransform(t_map_path);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
