#include "multi_map_navigation/multi_map_navigation.h"
#include <ros/package.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <multi_map_navigation/ShortestPath.h>
#include <multi_map_navigation/NavigateMapsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

namespace multi_map_navigation {

MultiMapNavigation::MultiMapNavigation(ros::NodeHandle& nh) : 
    nh_(nh),
    action_server_(nh_, "navigate_maps", boost::bind(&MultiMapNavigation::executeNavigation, this, _1), false),
    move_base_client_("move_base", true),
    current_map_(""),
    wormhole_db_(ros::package::getPath("multi_map_navigation") + "/wormholes.db") 
{
    // Initialize path planning service client
    path_client_ = nh_.serviceClient<multi_map_navigation::ShortestPath>("shortest_path");
    
    // Wait for move_base action server
    ROS_INFO("Waiting for move_base action server...");
    if (!move_base_client_.waitForServer(ros::Duration(5.0))) {
        ROS_WARN("move_base action server not available after waiting 5 seconds");
    }
    
    // Get the current map from a parameter
    if (!nh_.getParam("current_map", current_map_)) {
        ROS_WARN("No current_map parameter set, using empty string");
    }
    
    // Start the action server
    action_server_.start();
    ROS_INFO("Multi-map navigation action server is running");
}

MultiMapNavigation::~MultiMapNavigation() {
    // Nothing to do here, wormhole_db_ will be closed by its destructor
}

void MultiMapNavigation::executeNavigation(const multi_map_navigation::NavigateMapsGoalConstPtr& goal) {
    ROS_INFO("Received navigation goal to map '%s'", goal->target_map.c_str());
    
    bool success = false;
    
    // Check if the goal is in the current map
    if (goal->target_map == current_map_ || goal->target_map.empty()) {
        // Navigate within the current map
        success = navigateInMap(goal->target_pose);
    } else {
        // Navigate across maps
        success = navigateAcrossMaps(goal->target_map, goal->target_pose);
    }
    
    // Set action result
    multi_map_navigation::NavigateMapsResult result;
    result.success = success;
    
    if (success) {
        ROS_INFO("Navigation to map '%s' succeeded", goal->target_map.c_str());
        action_server_.setSucceeded(result);
    } else {
        ROS_ERROR("Navigation to map '%s' failed", goal->target_map.c_str());
        action_server_.setAborted(result);
    }
}

bool MultiMapNavigation::navigateInMap(const geometry_msgs::PoseStamped& goal) {
    ROS_INFO("Navigating within current map to position (%.2f, %.2f)", 
             goal.pose.position.x, goal.pose.position.y);
    
    // Create a move_base goal
    move_base_msgs::MoveBaseGoal move_base_goal;
    move_base_goal.target_pose = goal;
    
    // Send the goal to move_base
    move_base_client_.sendGoal(move_base_goal);
    
    // Wait for the result
    move_base_client_.waitForResult();
    
    // Check if the navigation was successful
    return move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

bool MultiMapNavigation::navigateAcrossMaps(const std::string& target_map, 
                                          const geometry_msgs::PoseStamped& target_pose) {
    ROS_INFO("Navigating from map '%s' to map '%s'", current_map_.c_str(), target_map.c_str());
    
    // Get the shortest path from the current map to the target map
    multi_map_navigation::ShortestPath srv;
    srv.request.start_map = current_map_;
    srv.request.target_map = target_map;
    
    if (!path_client_.call(srv)) {
        ROS_ERROR("Failed to call shortest_path service");
        return false;
    }
    
    if (srv.response.path.empty()) {
        ROS_ERROR("No path found from map '%s' to map '%s'", current_map_.c_str(), target_map.c_str());
        return false;
    }
    
    // Follow the path through the maps
    for (size_t i = 0; i < srv.response.path.size() - 1; ++i) {
        const std::string& from_map = srv.response.path[i];
        const std::string& to_map = srv.response.path[i + 1];
        
        ROS_INFO("Traversing from map '%s' to map '%s' (step %zu of %zu)",
                 from_map.c_str(), to_map.c_str(), i+1, srv.response.path.size()-1);
        
        // Get the wormhole between these maps
        WormholeDatabase::Wormhole wormhole = wormhole_db_.getWormholeBetween(from_map, to_map);
        
        if (wormhole.map_name.empty()) {
            ROS_ERROR("No wormhole found from map '%s' to map '%s'", from_map.c_str(), to_map.c_str());
            return false;
        }
        
        // Navigate to the wormhole position in the current map
        geometry_msgs::PoseStamped wormhole_pose;
        wormhole_pose.header.frame_id = "map";
        wormhole_pose.pose.position.x = wormhole.x;
        wormhole_pose.pose.position.y = wormhole.y;
        
        // Create a quaternion from the yaw angle
        tf2::Quaternion q;
        q.setRPY(0, 0, wormhole.yaw);
        wormhole_pose.pose.orientation.x = q.x();
        wormhole_pose.pose.orientation.y = q.y();
        wormhole_pose.pose.orientation.z = q.z();
        wormhole_pose.pose.orientation.w = q.w();
        
        ROS_INFO("Navigating to wormhole at (%.2f, %.2f, %.2f) in map '%s'",
                 wormhole.x, wormhole.y, wormhole.yaw, from_map.c_str());
        
        // Navigate to the wormhole position
        if (!navigateInMap(wormhole_pose)) {
            ROS_ERROR("Failed to navigate to wormhole in map '%s'", from_map.c_str());
            return false;
        }
        
        // Get the destination wormhole (entry point in the target map)
        WormholeDatabase::Wormhole entry_point = wormhole_db_.getWormholeBetween(to_map, from_map);
        
        if (entry_point.map_name.empty()) {
            ROS_ERROR("No entry point found in map '%s' from map '%s'", to_map.c_str(), from_map.c_str());
            return false;
        }
        
        // Create the initial pose for the new map
        geometry_msgs::Pose initial_pose;
        initial_pose.position.x = entry_point.x;
        initial_pose.position.y = entry_point.y;
        
        // Create orientation from yaw
        q.setRPY(0, 0, entry_point.yaw - 3.14159); // Adjust yaw for the new map
        initial_pose.orientation.x = q.x();
        initial_pose.orientation.y = q.y();
        initial_pose.orientation.z = q.z();
        initial_pose.orientation.w = q.w();
        
        // Switch maps using our launch file approach
        ROS_INFO("Switching map from '%s' to '%s'", from_map.c_str(), to_map.c_str());
        
        if (!switchMap(to_map)) {
            ROS_ERROR("Failed to switch map from '%s' to '%s'", from_map.c_str(), to_map.c_str());
            return false;
        }
        
        // Now, set the initial pose in AMCL after switching maps
        // Create a publisher for the initialpose topic that AMCL listens to
        ros::Publisher initial_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
        
        // Create the initial pose message
        geometry_msgs::PoseWithCovarianceStamped init_pose;
        init_pose.header.frame_id = "map";
        init_pose.header.stamp = ros::Time::now();
        init_pose.pose.pose = initial_pose;
        
        // Set a reasonable covariance
        for (int i = 0; i < 36; i++) {
            init_pose.pose.covariance[i] = 0.0;
        }
        // Set diagonal values for x, y, and yaw (positions 0, 7, and 35)
        init_pose.pose.covariance[0] = 0.25; // x
        init_pose.pose.covariance[7] = 0.25; // y
        init_pose.pose.covariance[35] = 0.06854; // yaw
        
        // Publish the initial pose
        ROS_INFO("Setting initial pose at (%.2f, %.2f, %.2f) in map '%s'",
                 initial_pose.position.x, initial_pose.position.y, 
                 tf2::getYaw(initial_pose.orientation), to_map.c_str());
        
        // Make sure the message gets published
        ros::Duration(0.5).sleep();
        initial_pose_pub.publish(init_pose);
        ros::Duration(0.5).sleep();
        
        ROS_INFO("Successfully switched to map '%s'", to_map.c_str());
    }
    
    // Navigate to the final goal in the target map
    ROS_INFO("Navigating to final goal in map '%s'", target_map.c_str());
    return navigateInMap(target_pose);
}

bool MultiMapNavigation::switchMap(const std::string& target_map) {
    ROS_INFO("Switching from map '%s' to map '%s'", current_map_.c_str(), target_map.c_str());
    
    // Get the map file path for the target map
    std::string map_file = ros::package::getPath("multi_map_navigation") + "/maps/" + target_map + ".yaml";
    
    // Check if the map file exists
    if (access(map_file.c_str(), F_OK) == -1) {
        ROS_ERROR("Map file '%s' does not exist", map_file.c_str());
        return false;
    }
    
    // Use system call to kill the current map_server and AMCL nodes
    system("rosnode kill /map_server /amcl &> /dev/null");
    
    // Give some time for the nodes to shut down
    ros::Duration(1.0).sleep();
    
    // Launch the new map using roslaunch
    std::string launch_cmd = "roslaunch multi_map_navigation map_navigation.launch map_file:=\"" + map_file + "\" map_name:=" + target_map + " &";
    ROS_INFO("Executing: %s", launch_cmd.c_str());
    int result = system(launch_cmd.c_str());
    
    if (result != 0) {
        ROS_ERROR("Failed to launch navigation with new map");
        return false;
    }
    
    // Update the current map
    current_map_ = target_map;
    
    // Update the parameter for other nodes
    nh_.setParam("current_map", current_map_);
    
    // Give time for the nodes to start
    ros::Duration(2.0).sleep();
    
    ROS_INFO("Switched to map '%s'", current_map_.c_str());
    return true;
}

} // namespace multi_map_navigation
