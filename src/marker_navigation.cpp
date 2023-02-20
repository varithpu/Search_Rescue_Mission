/**
 * @file marker_navigation.cpp
 * @author Varith Punturaumporn (varithpu@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2021-12-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <marker_navigation.h>

// create constructor with one argument
Marker_Navigation::Marker_Navigation(ros::NodeHandle *nodehandle):
    marker_nh{*nodehandle},
    vel_pub{},
    marker_sub{},
    tfBuffer{},
    tfListener(tfBuffer),
    marker_availble{false},
    marker_id{99},
    count_e{0},
    count_f{0},
    markerid_array{},
    target_pair{},
    markertf_array{},
    explorer_goal_sent{false},
    follower_goal_sent{false},
    explorer_done{false},
    explorer_backhome{false},
    follower_done{false},
    inspecting{false}
    {
        // get all four target location from param server
        marker_nh.getParam("/aruco_lookup_locations/target_1", target_1);
        ROS_ASSERT(target_1[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(target_1[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        target_pair.at(0).first = static_cast<double>(target_1[0]);
        target_pair.at(0).second = static_cast<double>(target_1[1]);

        marker_nh.getParam("/aruco_lookup_locations/target_2", target_2);
        ROS_ASSERT(target_2[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(target_2[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        target_pair.at(1).first = static_cast<double>(target_2[0]);
        target_pair.at(1).second = static_cast<double>(target_2[1]);

        marker_nh.getParam("/aruco_lookup_locations/target_3", target_3);
        ROS_ASSERT(target_3[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(target_3[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        target_pair.at(2).first = static_cast<double>(target_3[0]);
        target_pair.at(2).second = static_cast<double>(target_3[1]);

        marker_nh.getParam("/aruco_lookup_locations/target_4", target_4);
        ROS_ASSERT(target_4[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(target_4[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        target_pair.at(3).first = static_cast<double>(target_4[0]);
        target_pair.at(3).second = static_cast<double>(target_4[1]);

        // set explorer orientatin when the reached the target  
        target_orientation.setRPY(0,0,M_PI);

        // set rotational speed for marker inspection
        inspect_vel.linear.x = 0.0;
        inspect_vel.angular.z = -0.3;   

        // create publisher for sending velocity cammand during inspectiion 
        vel_pub = marker_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 1);

        // create subscriber to look for marker during inspection task
        marker_sub = marker_nh.subscribe("fiducial_transforms", 1, &Marker_Navigation::marker_callback, this);        
        }

void Marker_Navigation::set_marker_availble(bool input){
    marker_availble = input;
}

bool Marker_Navigation::get_marker_availble(){
    return marker_availble;
}

void Marker_Navigation::set_marker_id(int id){
    marker_id = id;
}

double Marker_Navigation::get_markertf_array(int index, TR tr){
    switch (tr)
    {
    case tx:
        return markertf_array.at(index).transform.translation.x;
        break;
    case ty:
        return markertf_array.at(index).transform.translation.y;
        break;
    case tz:
        return markertf_array.at(index).transform.translation.z;
        break;
    case qw:
        return markertf_array.at(index).transform.rotation.w;
        break;
    case qx:
        return markertf_array.at(index).transform.rotation.x;
        break;
    case qy:
        return markertf_array.at(index).transform.rotation.y;
        break;
    case qz:
        return markertf_array.at(index).transform.rotation.z;
        break;
    default: ROS_WARN_STREAM("enum input eror");
        break;
    }
}

std::pair<double, double> Marker_Navigation::get_target_pair(int index){
    return target_pair.at(index);
}

int Marker_Navigation::get_marker_id(){
    return marker_id;
}

void Marker_Navigation::set_markerid_array(int index, int value){
    markerid_array.at(index) = value;
}

int Marker_Navigation::get_markerid_array(int index){
    return markerid_array.at(index);
}

int Marker_Navigation::get_count_e() {
    return count_e;
}

void Marker_Navigation::count_e_plusone() {
    count_e += 1;
}

int Marker_Navigation::get_count_f() {
    return count_f;
}

void Marker_Navigation::count_f_plusone() {
    count_f += 1;
}

void Marker_Navigation::set_explorer_goal_sent(bool input){
    explorer_goal_sent = input;
}
bool Marker_Navigation::get_explorer_goal_sent(){
    return explorer_goal_sent;
}

void Marker_Navigation::set_follower_goal_sent(bool input){
    follower_goal_sent = input;
}

bool Marker_Navigation::get_follower_goal_sent(){
    return follower_goal_sent;
}

void Marker_Navigation::set_explorer_done(bool input){
    explorer_done = input;
}

bool Marker_Navigation::get_explorer_done(){
    return explorer_done;
}

void Marker_Navigation::set_explorer_backhome(bool input){
    explorer_backhome = input;
}

bool Marker_Navigation::get_explorer_backhome(){
    return explorer_backhome;
}

void Marker_Navigation::set_follower_done(bool input){
    follower_done = input;
}

bool Marker_Navigation::get_follower_done(){
    return follower_done;
}

void Marker_Navigation::set_inspecting(bool input){
    inspecting = input;
}

bool Marker_Navigation::get_inspecting(){
    return inspecting;
}

tf2::Quaternion Marker_Navigation::get_target_orientation(){
    return target_orientation;
}

void Marker_Navigation::inspect(){
    vel_pub.publish(inspect_vel);
}

void Marker_Navigation::listen() {
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
    if(count_e <= 4){
        // store the tf information in markertf_array
        markertf_array.at(markerid_array.at(count_e-1)).transform.translation.x = transformStamped.transform.translation.x;
        markertf_array.at(markerid_array.at(count_e-1)).transform.translation.y = transformStamped.transform.translation.y;
        markertf_array.at(markerid_array.at(count_e-1)).transform.translation.z = transformStamped.transform.translation.z;
        markertf_array.at(markerid_array.at(count_e-1)).transform.rotation.w = transformStamped.transform.rotation.w;
        markertf_array.at(markerid_array.at(count_e-1)).transform.rotation.x = transformStamped.transform.rotation.x;
        markertf_array.at(markerid_array.at(count_e-1)).transform.rotation.y = transformStamped.transform.rotation.y;
        markertf_array.at(markerid_array.at(count_e-1)).transform.rotation.z = transformStamped.transform.rotation.z;
    }
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

void Marker_Navigation::marker_callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
  if (!msg->transforms.empty()){ // check if the fiducial message is availble
    marker_availble = true; // set the marker_availbe to true of fiducial message is availble
    marker_id = msg->transforms.at(0).fiducial_id; // set marker_id attribute
    
    geometry_msgs::TransformStamped transformStamped; 
    static tf2_ros::TransformBroadcaster br;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
    transformStamped.child_frame_id = "marker_frame";

    // rotate the marker frame before publish so that x axis points towards the marker
    tf2::Quaternion q_old;
    tf2::Quaternion q_rot1;
    tf2::Quaternion q_rot2;
    tf2::Quaternion q_new;
    q_rot1.setRPY(0, M_PI/2, 0);
    q_rot2.setRPY(-M_PI/2, 0, 0);
    q_old.setW(msg->transforms.at(0).transform.rotation.w);
    q_old.setX(msg->transforms.at(0).transform.rotation.x);
    q_old.setY(msg->transforms.at(0).transform.rotation.y);
    q_old.setZ(msg->transforms.at(0).transform.rotation.z);
    q_new = (q_old*q_rot1)*q_rot2;
    q_new.normalize();

    // add offset from the wall and convert to camera frame
    tf2::Vector3 t_old(-0.4, 0, 0);
    auto t_new = tf2::quatRotate(q_new, t_old);

    // add offset to the transation and publish along with rotated marker frame
    transformStamped.transform.translation.x = msg->transforms.at(0).transform.translation.x + t_new.getX();
    transformStamped.transform.translation.y = msg->transforms.at(0).transform.translation.y + t_new.getY();
    transformStamped.transform.translation.z = msg->transforms.at(0).transform.translation.z + t_new.getZ();
    transformStamped.transform.rotation.w = q_new.getW();
    transformStamped.transform.rotation.x = q_new.getX();
    transformStamped.transform.rotation.y = q_new.getY();
    transformStamped.transform.rotation.z = q_new.getZ();

    //broadcast the new frame to /tf topic
    br.sendTransform(transformStamped);
  }else{
    marker_availble = false;
  }
}