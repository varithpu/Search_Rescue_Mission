/**
 * @file marker_navigation.h
 * @author Varith Punturaumporn (varithpu@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MARKER_NAVIGATION_H
#define MARKER_NAVIGATION_H

#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <utility_enum.h>

/**
 * @brief Marker_Navigation complements the job of explorer nad follower
 * It contains attributes to track the robot status and methods to subscribe marker information and publish commands to both robots
 * 
 */
class Marker_Navigation {

public:
    /**
     * @brief Construct a new Marker_Navigation object with one argument and initialize all attributes
     * 
     * @param nodehandle 
     */
    Marker_Navigation(ros::NodeHandle *nodehandle);
    
    /**
     * @brief Set the marker_availble attribute to indicate if the robot is now looking at the marker
     * 
     * @param input 
     */
    void set_marker_availble(bool input);

    /**
     * @brief Get the marker_availble attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_marker_availble();
    
    /**
     * @brief Set the marker_id attribute to keep track of the current marked id
     * 
     * @param id 
     */
    void set_marker_id(int id);

    /**
     * @brief Get the marker_id attribute
     * 
     * @return int 
     */
    int get_marker_id();

    /**
     * @brief Set the markerid array to track the sequence of visited markers
     * 
     * @param index 
     * @param value 
     */
    void set_markerid_array(int index, int value);

    /**
     * @brief Get the markerid array 
     * 
     * @param index 
     * @return int 
     */
    int get_markerid_array(int index);

    /**
     * @brief Get the target pair to tell where the explorer should visit 
     * 
     * @param index 
     * @return std::pair<double, double> 
     */
    std::pair<double, double> get_target_pair(int index);

    /**
     * @brief Get the markertf array to set the goal for the follower
     * 
     * @param index 
     * @param tr 
     * @return double 
     */
    double get_markertf_array(int index, TR tr);    

    /**
     * @brief Get the count_e attribute to track the number of visited markers by the explorer 
     * 
     * @return int 
     */
    int get_count_e();

    /**
     * @brief add one to the count_e attribute once the marker is found by explorer
     * 
     */
    void count_e_plusone();
    
    /**
     * @brief Get the count_f attribute to track the number of visited markers by the follower 
     * 
     * @return int 
     */
    int get_count_f();

    /**
     * @brief add one to the count_f attribute once the marker is visited by the follower
     * 
     */
    void count_f_plusone();

    /**
     * @brief Set the explorer_goal_sent attribute to indicate that sending goal to exploror is done
     * 
     * @param input 
     */
    void set_explorer_goal_sent(bool input);

    /**
     * @brief Get the explorer_goal_sent attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_explorer_goal_sent();

    /**
     * @brief Set the follower_goal_sent attribute to indicate that sending goal to follower is done
     * 
     * @param input 
     */
    void set_follower_goal_sent(bool input);

    /**
     * @brief Get the follower_goal_sent attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_follower_goal_sent();

    /**
     * @brief Set the explorer_done attribute to indicate that all explorer jobs are done
     * 
     * @param input 
     */
    void set_explorer_done(bool input);

    /**
     * @brief Get the explorer_done attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_explorer_done();

    /**
     * @brief Set the explorer_backhome attribute to indicate that the explorer came back home successfully
     * 
     * @param input 
     */
    void set_explorer_backhome(bool input);

    /**
     * @brief Get the explorer_backhome attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_explorer_backhome();

    /**
     * @brief Set the follower_done attribute to indicate that all follower jobs are done
     * 
     * @param input 
     */
    void set_follower_done(bool input);

    /**
     * @brief Get the follower_done attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_follower_done();

    /**
     * @brief Set the inspecting attribute to indicate if the robot is now doing the marker inspection
     * 
     * @param input 
     */
    void set_inspecting(bool input);

    /**
     * @brief Get the inspecting attribute
     * 
     * @return true 
     * @return false 
     */
    bool get_inspecting();

    /**
     * @brief Get the explorer desired orientation when reached the target
     * 
     * @return tf2::Quaternion 
     */
    tf2::Quaternion get_target_orientation();

    /**
     * @brief callback function to get the fiducial message from the marker
     * 
     * @param msg 
     */
    void marker_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    /**
     * @brief listen function extract the tf bween marker and map from the tf topic
     * 
     */
    void listen();

    /**
     * @brief inspect function publishes the velocity command to the explorer to rotate while doing tinspection
     * 
     */
    void inspect();

private:
    ros::NodeHandle marker_nh; // node handle initiated in main which pass through the constructor as pointer
    ros::Publisher vel_pub; //  velocity command pubhisher for the explorer to rotate during inspection
    ros::Subscriber marker_sub; // subscriber to extrract information from ficucial_transforms topic
    bool marker_availble; // status to incicate whether the marker is now availbe on camera
    int marker_id; //  tracks the most recent vesited marker id 
    int count_e = 0; // counts the number of markers visited by the explorer
    int count_f = 0; // counts the number of markers visited by the follower
    std::array<int,4> markerid_array; // array for storing the sequence of visited marker id
    std::array <std::pair<double, double>, 4> target_pair; // array for storing location of targets
    std::array <geometry_msgs::TransformStamped,4> markertf_array; // array for storing tf between marker and map 
    bool explorer_goal_sent; // status to indicate that the goal for explorer was sent
    bool follower_goal_sent; // status to indicate that the goal for follower was sent
    bool explorer_done; // status to indicate that all exploror jobs are done
    bool explorer_backhome; // status to indicate explorer came back home succesfully
    bool follower_done; // status to indicate that all exploror jobs are done
    bool inspecting; // status to indicate that the robot is now doing the marker inspection
    XmlRpc::XmlRpcValue target_1; // target location 1
    XmlRpc::XmlRpcValue target_2; // target location 2
    XmlRpc::XmlRpcValue target_3; // target location 3
    XmlRpc::XmlRpcValue target_4; // target location 4
    tf2::Quaternion target_orientation; // desired orientation of the explorer at the target
    geometry_msgs::Twist inspect_vel; // velocity message for explorer inspection
    tf2_ros::Buffer tfBuffer; // buffer for containing tf
    tf2_ros::TransformListener tfListener; // listener to look for tf in the buffer
};
#endif