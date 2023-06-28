#ifndef _READ_PATH_HPP_
#define _READ_PATH_HPP_


#include <ctime>
#include <time.h>
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include "geometry_msgs/Quaternion.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <upo_actions/NavigateAction.h>



using namespace std;

class ReadPath
{

    typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;

public:
    ReadPath();
    ReadPath(std::string name_);
    void CreatePath();
    void readPathCallBack(const visualization_msgs::Marker::ConstPtr &msg_);
    geometry_msgs::Quaternion getOrientation(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_);
    void configServices();
    void ManagePath();

    std::unique_ptr<NavigateClient> NavigationClient; // For UGV
    upo_actions::NavigateGoal ugv_goal3D;

    ros::NodeHandlePtr nh;
    ros::Subscriber read_path_sub;
    vector<geometry_msgs::Vector3> v_path;

    double dis_min_save_path_point, time_max;
    string ros_node_name;
    bool use_create_file, sent_new_ugv_wp, is_ugv_in_waypoint;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory;
	trajectory_msgs::MultiDOFJointTrajectory traj;
    int num_wp;
    ros::Time time_count_ugv;

};

#endif
