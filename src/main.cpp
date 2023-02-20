/**
 * @file main.cpp
 * @author Varith Punturaumporn (varithpu@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <marker_navigation.h>
#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) 
{
  // initialize node and create node handle
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  // create object of class Marker_Navigation
  Marker_Navigation nav(&nh);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);
  // tell the action client that we want to spin a thread by default
  MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }
  // initialize explorer and follower goal
  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;
  
  // Set explorer MoveBase header
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();

  // Set follower MoveBase header
  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now(); 
  
  ros::Rate loop_rate(10);
  
  // begin the task 
  while (ros::ok()) {
    // explorer inspect the marker when it reaches the target location
    while((nav.get_inspecting() == true) && (nav.get_count_e() < 4)){
      nav.inspect();
      // if the marker is found, collect the marker id and exit the inspection loop 
      if(nav.get_marker_availble() == true){
        nav.set_markerid_array(nav.get_count_e(), nav.get_marker_id());
        ROS_INFO_STREAM("ID : " << nav.get_marker_id(););
        nav.set_inspecting(false);
        nav.set_marker_availble(false);
        nav.set_explorer_goal_sent(false);
        nav.count_e_plusone();
        break;
      }
      ros::spinOnce(); // call callback function only when inspecting the marker
    }
    // sent target location to the explorer 
    if ((!nav.get_explorer_goal_sent()) && (nav.get_count_e() < 4)) {
      ROS_INFO("Sending goal for explorer");
      explorer_goal.target_pose.pose.position.x = nav.get_target_pair(nav.get_count_e()).first;
      explorer_goal.target_pose.pose.position.y = nav.get_target_pair(nav.get_count_e()).second;
      explorer_goal.target_pose.pose.orientation.w = nav.get_target_orientation().getW();
      explorer_goal.target_pose.pose.orientation.x = nav.get_target_orientation().getX();
      explorer_goal.target_pose.pose.orientation.y = nav.get_target_orientation().getY();
      explorer_goal.target_pose.pose.orientation.z = nav.get_target_orientation().getZ();
      explorer_client.sendGoal(explorer_goal);
      nav.set_explorer_goal_sent(true);
    }
    // tell the explorer to inspect the marker when it reached the goal
    if ((explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (nav.get_count_e() < 4)) {
      ROS_INFO("Hooray, exploror reached goal");
      nav.set_inspecting(true);
    }
    // once visited all targets, tell the exploror to come back home
    if((nav.get_count_e() == 4) && (nav.get_explorer_done() == false)){
      ROS_INFO("Explorer coming home");
      explorer_goal.target_pose.pose.position.x = -4.0;
      explorer_goal.target_pose.pose.position.y = 2.5;
      explorer_goal.target_pose.pose.orientation.w = 1.0;
      explorer_goal.target_pose.pose.orientation.x = 0.0;
      explorer_goal.target_pose.pose.orientation.y = 0.0;
      explorer_goal.target_pose.pose.orientation.z = 0.0;
      explorer_client.sendGoal(explorer_goal);
      nav.set_explorer_done(true);
    }
    // when the exploror reached home, start the follower task
    if ((explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (nav.get_explorer_done() == true) && (nav.get_explorer_backhome() == false)) {
      ROS_INFO("Hooray, exploror reached home");
      nav.set_explorer_backhome(true);
    }
    if (nav.get_explorer_backhome() == true){
      // sent goal location to the follower 
      if ((!nav.get_follower_goal_sent()) && (nav.get_count_f() < 4)) {
        ROS_INFO("Sending goal for follower");
        follower_goal.target_pose.pose.position.x = nav.get_markertf_array(nav.get_count_f(), tx);
        follower_goal.target_pose.pose.position.y = nav.get_markertf_array(nav.get_count_f(), ty);
        follower_goal.target_pose.pose.orientation.w = nav.get_markertf_array(nav.get_count_f(), qw);
        follower_goal.target_pose.pose.orientation.x = nav.get_markertf_array(nav.get_count_f(), qx);
        follower_goal.target_pose.pose.orientation.y = nav.get_markertf_array(nav.get_count_f(), qy);
        follower_goal.target_pose.pose.orientation.z = nav.get_markertf_array(nav.get_count_f(), qz);
        follower_client.sendGoal(follower_goal);
        nav.set_follower_goal_sent(true);
      }
      // when the follower reaches a goal, sent the location of the next goal
      if ((follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (nav.get_count_f() < 4)) {
        ROS_INFO("Hooray, follower reached goal");
        nav.set_follower_goal_sent(false);
        nav.count_f_plusone();
      }
      // once visited all targets, tell the follwer to come back home
      if((nav.get_count_f() == 4) && (nav.get_follower_done() == false)){
        ROS_INFO("Follower coming home");
        follower_goal.target_pose.pose.position.x = -4.0;
        follower_goal.target_pose.pose.position.y = 3.5;
        follower_goal.target_pose.pose.orientation.w = 1.0;
        follower_goal.target_pose.pose.orientation.x = 0.0;
        follower_goal.target_pose.pose.orientation.y = 0.0;
        follower_goal.target_pose.pose.orientation.z = 0.0;
        follower_client.sendGoal(follower_goal);
        nav.set_follower_done(true);
      }
      // shutdown the node once the follower reached home
      if ((follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (nav.get_follower_done() == true)) {
        ROS_INFO("All done, Good Job !");
        ros::shutdown();
      }
    }
    nav.listen(); // call the listen function to look for tf between marker and map 
    loop_rate.sleep();
  }
}
