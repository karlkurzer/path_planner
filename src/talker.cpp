#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <vector>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("current_goal",10);
    ros::Publisher plan_pub = n.advertise<nav_msgs::Path>("current_path",10);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "/goal";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = 0;
        goal.pose.position.y = 0;
        goal.pose.position.z = 0;
        goal.pose.orientation.x = 0;
        goal.pose.orientation.y = 0;
        goal.pose.orientation.z = 0;
        goal.pose.orientation.w = 0;
        goal_pub.publish(goal);

        std::vector<geometry_msgs::PoseStamped> p (3);
        p[0].pose.position.x = 1;
        p[0].pose.position.y = 0;
        p[0].pose.position.z = 0;
        p[0].pose.orientation.x = 0;
        p[0].pose.orientation.y = 0;
        p[0].pose.orientation.z = 0;
        p[0].pose.orientation.w = 0;

        p[1].pose.position.x = 2;
        p[1].pose.position.y = 0;
        p[1].pose.position.z = 0;
        p[1].pose.orientation.x = 0;
        p[1].pose.orientation.y = 0;
        p[1].pose.orientation.z = 0;
        p[1].pose.orientation.w = 0;

        p[2].pose.position.x = 2.4;
        p[2].pose.position.y = 3;
        p[2].pose.position.z = 0;
        p[2].pose.orientation.x = 0;
        p[2].pose.orientation.y = 0;
        p[2].pose.orientation.z = 0;
        p[2].pose.orientation.w = 0;

        nav_msgs::Path plan;
        plan.header.frame_id = "/plan";
        plan.header.stamp = ros::Time::now();
        plan.poses = p;
        plan_pub.publish(plan);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
