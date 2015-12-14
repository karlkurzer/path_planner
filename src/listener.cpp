#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    for (int i = 0; i < msg->data.size(); i++)
    {
        std::cout <<std::to_string(msg->data[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);
    ros::spin();
    return 0;
}
