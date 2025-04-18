#ifndef MULTI_MAP_NAVIGATION_H
#define MULTI_MAP_NAVIGATION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multi_map_navigation/NavigateMapsAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <string>
#include <unistd.h> // For access() function

#include "multi_map_navigation/wormhole_database.h"
#include "multi_map_navigation/map_graph.h"

namespace multi_map_navigation {

/**
 * @brief Multi-map navigation action server
 * 
 * This class provides an action server for navigating between different maps
 * using wormholes. It can handle navigation goals within the same map or
 * across multiple maps.
 */
class MultiMapNavigation {
public:
    /**
     * @brief Constructor
     * @param nh ROS NodeHandle
     */
    MultiMapNavigation(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~MultiMapNavigation();

private:
    ros::NodeHandle& nh_;
    
    // Action server for multi-map navigation requests
    actionlib::SimpleActionServer<multi_map_navigation::NavigateMapsAction> action_server_;
    
    // Client for move_base actions
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    
    // Current map of the robot
    std::string current_map_;
    
    // Database for wormholes
    WormholeDatabase wormhole_db_;
    
    // Service client for shortest path queries
    ros::ServiceClient path_client_;

    /**
     * @brief Callback for the navigation action server
     * @param goal Goal with target position and map
     */
    void executeNavigation(const multi_map_navigation::NavigateMapsGoalConstPtr& goal);
    
    /**
     * @brief Navigate to a position in the current map
     * @param goal Target position
     * @return bool True if successful, false otherwise
     */
    bool navigateInMap(const geometry_msgs::PoseStamped& goal);
    
    /**
     * @brief Navigate to a different map using wormholes
     * @param target_map Target map name
     * @param target_pose Target pose in the target map
     * @return bool True if successful, false otherwise
     */
    bool navigateAcrossMaps(const std::string& target_map, const geometry_msgs::PoseStamped& target_pose);
    
    /**
     * @brief Switch to a different map
     * @param target_map Target map name
     * @return bool True if successful, false otherwise
     */
    bool switchMap(const std::string& target_map);
};

} // namespace multi_map_navigation

#endif // MULTI_MAP_NAVIGATION_H
