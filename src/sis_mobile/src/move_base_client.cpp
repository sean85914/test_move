#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_client");
  
  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.707;
  goal.target_pose.pose.orientation.w = 0.707;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult(ros::Duration(5.0));
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Succeeded");
  else
    ROS_INFO("Failed");
  return 0;
}
