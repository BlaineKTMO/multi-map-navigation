#include "multi_map_navigation/shortest_path_service.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "shortest_path_service_node");
    ros::NodeHandle nh;
    
    // Create the shortest path service
    multi_map_navigation::ShortestPathService shortest_path_service(nh);
    
    ROS_INFO("Shortest path service node is running");
    
    // Spin to process callbacks
    ros::spin();
    
    return 0;
}
