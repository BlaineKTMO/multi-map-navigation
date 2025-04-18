#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <multi_map_navigation/NavigateMapsAction.h>
#include <geometry_msgs/PoseStamped.h>

// Typedef for convenience
typedef actionlib::SimpleActionClient<multi_map_navigation::NavigateMapsAction> NavigationClient;

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_navigator");
  ros::NodeHandle nh;
  
  // Get parameters from parameter server or command line
  std::string target_map;
  double x, y, z, qx, qy, qz, qw;
  
  ros::param::param<std::string>("~target_map", target_map, "mapA");
  ros::param::param<double>("~x", x, -4.0);
  ros::param::param<double>("~y", y, -5.0);
  ros::param::param<double>("~z", z, 0.0);
  ros::param::param<double>("~qx", qx, 0.0);
  ros::param::param<double>("~qy", qy, 0.0);
  ros::param::param<double>("~qz", qz, 0.0);
  ros::param::param<double>("~qw", qw, 1.0);
  
  ROS_INFO("Creating action client for navigate_maps server...");
  NavigationClient client("navigate_maps", true);
  
  ROS_INFO("Waiting for action server to start...");
  if (!client.waitForServer(ros::Duration(10.0))) {
    ROS_ERROR("Action server did not start within timeout!");
    return 1;
  }
  
  ROS_INFO("Action server started. Sending goal...");

  // Create a goal
  multi_map_navigation::NavigateMapsGoal goal;
  goal.target_map = target_map;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;
  goal.target_pose.pose.orientation.x = qx;
  goal.target_pose.pose.orientation.y = qy;
  goal.target_pose.pose.orientation.z = qz;
  goal.target_pose.pose.orientation.w = qw;
  
  ROS_INFO("Sending navigation goal to map '%s' at position (%.2f, %.2f)", 
          target_map.c_str(), x, y);
  
  // Send the goal
  client.sendGoal(goal, &doneCb, NULL, &feedbackCb);
  
  // Wait for the action to complete
  ROS_INFO("Waiting for result...");
  if (client.waitForResult(ros::Duration(60.0))) {
    ROS_INFO("Goal completed with result: %s", 
             client.getState().toString().c_str());
  } else {
    ROS_WARN("Goal did not complete within timeout");
  }
  
  return 0;
}
