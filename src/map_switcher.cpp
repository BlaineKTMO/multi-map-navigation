#include "multi_map_navigation/map_switcher.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <multi_map_navigation/LoadMap.h>
#include <ros/package.h>

namespace multi_map_navigation {

MapSwitcher::MapSwitcher(ros::NodeHandle& nh) : 
    nh_(nh),
    action_server_(nh_, "switch_map", boost::bind(&MapSwitcher::executeSwitchMap, this, _1), false),
    tf_listener_(tf_buffer_),
    current_map_("")
{
    // Initialize service clients
    map_unload_client_ = nh_.serviceClient<std_srvs::Empty>("/map_server/unload_map");
    map_load_client_ = nh_.serviceClient<multi_map_navigation::LoadMap>("/map_server/load_map");
    set_pose_client_ = nh_.serviceClient<std_srvs::Empty>("/initialpose");
    
    // Initialize the publisher for initial pose
    initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
    
    // Get the current map from a parameter
    if (!nh_.getParam("current_map", current_map_)) {
        ROS_WARN("No current_map parameter set, using empty string");
    }
    
    // Start the action server
    action_server_.start();
    ROS_INFO("Map switcher action server is running");
}

MapSwitcher::~MapSwitcher() {
    // No cleanup needed
}

void MapSwitcher::executeSwitchMap(const multi_map_navigation::SwitchMapGoalConstPtr& goal) {
    ROS_INFO("Received request to switch from map '%s' to map '%s'", 
             goal->current_map.c_str(), goal->target_map.c_str());
    
    // Set up result and feedback messages
    multi_map_navigation::SwitchMapResult result;
    multi_map_navigation::SwitchMapFeedback feedback;
    
    // Check if we're already in the target map
    if (goal->current_map == goal->target_map) {
        ROS_INFO("Already in the target map '%s', nothing to do", goal->target_map.c_str());
        result.success = true;
        result.message = "Already in target map";
        action_server_.setSucceeded(result);
        return;
    }
    
    // Update progress
    feedback.status = "Unloading current map";
    feedback.percent_complete = 0.0;
    action_server_.publishFeedback(feedback);
    
    // Unload the current map
    if (!unloadMap()) {
        result.success = false;
        result.message = "Failed to unload the current map";
        action_server_.setAborted(result);
        return;
    }
    
    // Update progress
    feedback.status = "Loading new map";
    feedback.percent_complete = 33.0;
    action_server_.publishFeedback(feedback);
    
    // Load the target map
    if (!loadMap(goal->target_map)) {
        result.success = false;
        result.message = "Failed to load the target map";
        action_server_.setAborted(result);
        return;
    }
    
    // Update progress
    feedback.status = "Setting initial pose";
    feedback.percent_complete = 66.0;
    action_server_.publishFeedback(feedback);
    
    // Set the initial pose in the new map
    if (!setInitialPose(goal->initial_pose.position.x, 
                        goal->initial_pose.position.y,
                        tf2::getYaw(goal->initial_pose.orientation))) {
        result.success = false;
        result.message = "Failed to set initial pose";
        action_server_.setAborted(result);
        return;
    }
    
    // Update the current map parameter
    current_map_ = goal->target_map;
    nh_.setParam("current_map", current_map_);
    
    // Complete the action
    feedback.status = "Map switch complete";
    feedback.percent_complete = 100.0;
    action_server_.publishFeedback(feedback);
    
    result.success = true;
    result.message = "Successfully switched to map " + goal->target_map;
    action_server_.setSucceeded(result);
    
    ROS_INFO("Successfully switched from map '%s' to map '%s'", 
             goal->current_map.c_str(), goal->target_map.c_str());
}

bool MapSwitcher::unloadMap() {
    ROS_INFO("Unloading current map");
    
    std_srvs::Empty srv;
    if (!map_unload_client_.call(srv)) {
        ROS_ERROR("Failed to call map unload service");
        return false;
    }
    
    // Add a short delay to allow the map to unload
    ros::Duration(0.5).sleep();
    return true;
}

bool MapSwitcher::loadMap(const std::string& map_name) {
    ROS_INFO("Loading map '%s'", map_name.c_str());
    
    // Construct the path to the map file
    std::string map_path = ros::package::getPath("multi_map_navigation") + "/maps/" + map_name + ".yaml";
    
    multi_map_navigation::LoadMap srv;
    srv.request.map_path = map_path;
    
    if (!map_load_client_.call(srv)) {
        ROS_ERROR("Failed to call map load service for map '%s' at path '%s'", 
                  map_name.c_str(), map_path.c_str());
        return false;
    }
    
    // Add a short delay to allow the map to load
    ros::Duration(0.5).sleep();
    return true;
}

bool MapSwitcher::setInitialPose(double x, double y, double yaw) {
    ROS_INFO("Getting current robot pose before setting initial pose in new map");
    
    // Create the pose message
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    
    // Try to get the current robot pose from TF
    try {
        // Look up the transform from map to base_footprint
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
        
        // Use the current robot pose instead of the provided coordinates
        pose.pose.pose.position.x = transform.transform.translation.x;
        pose.pose.pose.position.y = transform.transform.translation.y;
        pose.pose.pose.position.z = transform.transform.translation.z;
        pose.pose.pose.orientation = transform.transform.rotation;
        
        ROS_INFO("Using current robot pose at (%.2f, %.2f)", 
                pose.pose.pose.position.x, 
                pose.pose.pose.position.y);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to get current robot pose: %s", ex.what());
        ROS_WARN("Falling back to provided coordinates");
        
        // Fall back to the provided coordinates
        pose.pose.pose.position.x = x;
        pose.pose.pose.position.y = y;
        pose.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.pose.orientation.x = q.x();
        pose.pose.pose.orientation.y = q.y();
        pose.pose.pose.orientation.z = q.z();
        pose.pose.pose.orientation.w = q.w();
    }
    
    // Set a very small covariance to force AMCL to reset its belief
    // This is critical for proper re-localization after map switching
    for (int i = 0; i < 36; ++i) {
        pose.pose.covariance[i] = 0.0;
    }
    pose.pose.covariance[0] = 0.01;  // x position variance (meters^2)
    pose.pose.covariance[7] = 0.01;  // y position variance (meters^2)
    pose.pose.covariance[35] = 0.01; // yaw angle variance (radians^2)
    
    // Ensure AMCL has started before sending initial pose
    ros::Duration(1.0).sleep();
    
    // Publish the initial pose multiple times to make sure AMCL receives it
    for (int i = 0; i < 5; ++i) {
        initial_pose_pub_.publish(pose);
        ros::Duration(0.2).sleep();
    }
    
    // Wait a bit longer to make sure AMCL has processed the pose
    ros::Duration(0.5).sleep();
    
    ROS_INFO("Initial pose has been set");
    return true;
}

} // namespace multi_map_navigation
