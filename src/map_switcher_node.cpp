#include "multi_map_navigation/map_switcher.h"
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_switcher_node");
    ros::NodeHandle nh;
    
    // Create the map switcher
    multi_map_navigation::MapSwitcher map_switcher(nh);
    
    ROS_INFO("Map switcher node is running");
    
    // Spin to process callbacks
    ros::spin();
    
    return 0;
}
