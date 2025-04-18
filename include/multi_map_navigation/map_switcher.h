#ifndef MAP_SWITCHER_H
#define MAP_SWITCHER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <multi_map_navigation/SwitchMapAction.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace multi_map_navigation {

/**
 * @brief Action server for switching between maps
 * 
 * This class provides an action server that handles map switching operations.
 * It navigates to the wormhole, unloads the current map, loads the new map,
 * and sets the robot's initial position.
 */
class MapSwitcher {
public:
    /**
     * @brief Constructor
     * @param nh ROS NodeHandle
     */
    MapSwitcher(ros::NodeHandle& nh);
    
    /**
     * @brief Destructor
     */
    ~MapSwitcher();

private:
    ros::NodeHandle& nh_;
    
    // Action server for map switching
    actionlib::SimpleActionServer<multi_map_navigation::SwitchMapAction> action_server_;
    
    // Service clients for map operations
    ros::ServiceClient map_unload_client_;
    ros::ServiceClient map_load_client_;
    ros::ServiceClient set_pose_client_;
    
    // Publisher for initial pose
    ros::Publisher initial_pose_pub_;
    
    // TF2 for pose tracking
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    std::string current_map_;
    
    /**
     * @brief Callback for the switch map action server
     * @param goal Goal with target map and position
     */
    void executeSwitchMap(const multi_map_navigation::SwitchMapGoalConstPtr& goal);
    
    /**
     * @brief Unload the current map
     * @return bool True if successful, false otherwise
     */
    bool unloadMap();
    
    /**
     * @brief Load a new map
     * @param map_name Name of the map to load
     * @return bool True if successful, false otherwise
     */
    bool loadMap(const std::string& map_name);
    
    /**
     * @brief Set the robot's initial pose in the new map
     * @param x X coordinate
     * @param y Y coordinate
     * @param yaw Yaw orientation
     * @return bool True if successful, false otherwise
     */
    bool setInitialPose(double x, double y, double yaw);
};

} // namespace multi_map_navigation

#endif // MAP_SWITCHER_H
