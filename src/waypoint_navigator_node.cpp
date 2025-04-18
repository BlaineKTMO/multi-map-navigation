#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_navigation/NavigateMapsAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

// Typedef for convenience
typedef actionlib::SimpleActionClient<multi_map_navigation::NavigateMapsAction> NavigationClient;

// Global variables to store the starting position
std::string start_map;
geometry_msgs::PoseStamped start_pose;

// Callback for when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const multi_map_navigation::NavigateMapsResultConstPtr& result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Navigation succeeded!");
  } else {
    ROS_WARN("Navigation failed with state: %s", state.toString().c_str());
  }
}

// Callback for feedback during goal execution
void feedbackCb(const multi_map_navigation::NavigateMapsFeedbackConstPtr& feedback)
{
  ROS_INFO("Navigation feedback: Map: %s, Status: %s, Progress: %.2f%%", 
           feedback->current_map.c_str(), 
           feedback->status.c_str(), 
           feedback->percent_complete);
}

// Function to get the current map
std::string getCurrentMap() {
  std::string current_map;
  if (!ros::param::get("current_map", current_map)) {
    ROS_WARN("Failed to get current map parameter, defaulting to 'mapA'");
    current_map = "mapA";
  }
  return current_map;
}

// Function to get the current robot pose
geometry_msgs::PoseStamped getCurrentPose() {
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::PoseStamped pose;
  
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  
  try {
    geometry_msgs::TransformStamped transform = 
        tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
    
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    
    ROS_INFO("Current robot pose: (%.2f, %.2f)", pose.pose.position.x, pose.pose.position.y);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Failed to get robot pose: %s", ex.what());
    // Provide a default pose in case of failure
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
  }
  
  return pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_navigator");
  ros::NodeHandle nh;
  
  // Allow ROS time to initialize
  ros::Duration(1.0).sleep();
  
  // Save the initial position and map
  start_map = getCurrentMap();
  start_pose = getCurrentPose();
  
  ROS_INFO("Saved initial position in map '%s' at (%.2f, %.2f)", 
           start_map.c_str(), start_pose.pose.position.x, start_pose.pose.position.y);
  
  // Get parameters from parameter server or command line
  std::string target_map;
  double x, y, z, qx, qy, qz, qw;
  bool return_to_start;
  
  ros::param::param<std::string>("~target_map", target_map, "mapA");
  ros::param::param<double>("~x", x, -4.0);
  ros::param::param<double>("~y", y, -5.0);
  ros::param::param<double>("~z", z, 0.0);
  ros::param::param<double>("~qx", qx, 0.0);
  ros::param::param<double>("~qy", qy, 0.0);
  ros::param::param<double>("~qz", qz, 0.0);
  ros::param::param<double>("~qw", qw, 1.0);
  ros::param::param<bool>("~return_to_start", return_to_start, true);
  
  ROS_INFO("Creating action client for navigate_maps server...");
  NavigationClient client("navigate_maps", true);
  
  ROS_INFO("Waiting for action server to start...");
  if (!client.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Action server did not start within timeout!");
    return 1;
  }
  
  ROS_INFO("Action server started. Sending goal to destination...");

  // Create a goal to the destination
  multi_map_navigation::NavigateMapsGoal destination_goal;
  destination_goal.target_map = target_map;
  destination_goal.target_pose.header.frame_id = "map";
  destination_goal.target_pose.header.stamp = ros::Time::now();
  destination_goal.target_pose.pose.position.x = x;
  destination_goal.target_pose.pose.position.y = y;
  destination_goal.target_pose.pose.position.z = z;
  destination_goal.target_pose.pose.orientation.x = qx;
  destination_goal.target_pose.pose.orientation.y = qy;
  destination_goal.target_pose.pose.orientation.z = qz;
  destination_goal.target_pose.pose.orientation.w = qw;
  
  ROS_INFO("Sending navigation goal to map '%s' at position (%.2f, %.2f)", 
          target_map.c_str(), x, y);
  
  // Send the destination goal
  client.sendGoal(destination_goal, &doneCb, NULL, &feedbackCb);
  
  // Wait for the destination action to complete
  ROS_INFO("Waiting for destination goal to complete...");
  bool destination_succeeded = false;
  if (client.waitForResult(ros::Duration(60.0))) {
    destination_succeeded = (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    ROS_INFO("Destination goal completed with result: %s", 
             client.getState().toString().c_str());
  } else {
    ROS_WARN("Destination goal did not complete within timeout");
  }
  
  // If return to start is enabled and destination was reached, navigate back to the starting position
  if (return_to_start) {
    ROS_INFO("Preparing to return to starting position in map '%s' at (%.2f, %.2f)", 
            start_map.c_str(), start_pose.pose.position.x, start_pose.pose.position.y);
    
    // Create return goal
    multi_map_navigation::NavigateMapsGoal return_goal;
    return_goal.target_map = start_map;
    return_goal.target_pose.header = start_pose.header;
    return_goal.target_pose.pose = start_pose.pose;
    
    // A short pause before returning
    ros::Duration(2.0).sleep();
    
    ROS_INFO("Sending navigation goal to return to starting position...");
    client.sendGoal(return_goal, &doneCb, NULL, &feedbackCb);
    
    // Wait for the return action to complete
    ROS_INFO("Waiting for return journey to complete...");
    if (client.waitForResult(ros::Duration(60.0))) {
      ROS_INFO("Return journey completed with result: %s", 
               client.getState().toString().c_str());
    } else {
      ROS_WARN("Return journey did not complete within timeout");
    }
  } else {
    ROS_INFO("Return to start is disabled. Navigation complete.");
  }
  
  return 0;
}
