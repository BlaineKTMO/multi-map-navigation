#include "multi_map_navigation/multi_map_navigation.h"
#include <ros/ros.h>

using namespace multi_map_navigation;

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_map_navigation_node");
    ros::NodeHandle nh;

    // Create an instance of the MultiMapNavigation class
    MultiMapNavigation multi_map_navigation(nh);

    ROS_INFO("Multi-Map Navigation Node is running.");

    ros::spin();
    return 0;
}
