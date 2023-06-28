#include "ugv_path_tracker/read_path.hpp"

ReadPath::ReadPath(std::string name_)
{
    nh.reset(new ros::NodeHandle("~"));

    ros_node_name = name_;

    nh->param("time_max", time_max, (double)20);
    nh->param("dis_min_save_path_point", dis_min_save_path_point, (double)0.2);
    nh->param<bool>("use_create_file",use_create_file, false);
    nh->param<bool>("is_ugv_in_waypoint",is_ugv_in_waypoint, false);

    read_path_sub = nh->subscribe("/planner_ros_node/path_points_markers", 1, &ReadPath::readPathCallBack, this);
    configServices();
}

void ReadPath::readPathCallBack(const visualization_msgs::Marker::ConstPtr &msg_)
{
    v_path.clear();
    num_wp = 0;

    geometry_msgs::Vector3 p_;

    for (size_t i=0 ; i< msg_->points.size(); i++ ){
        p_.x = msg_->points[i].x;
        p_.y = msg_->points[i].y;
        p_.z = msg_->points[i].z;
        if (i < msg_->points.size() -1){
            double d1_ = sqrt ( pow(msg_->points[i+1].x - msg_->points[i].x,2) + 
                                pow(msg_->points[i+1].y - msg_->points[i].y,2) + 
                                pow(msg_->points[i+1].z - msg_->points[i].z,2) );
            if (d1_ > dis_min_save_path_point)
                v_path.push_back(p_);
        } else{
                v_path.push_back(p_);
        }
        std::cout<<"Original path["<< i <<"] : ["<< msg_->points[i].x <<" , "<< msg_->points[i].y <<" , "<< msg_->points[i].z <<"]" <<std::endl;
    }

    for (size_t i=0 ; i< v_path.size(); i++ ){
        std::cout<<"New path : ["<< i <<"] : ["<< v_path[i].x <<" , "<< v_path[i].y <<" , "<< v_path[i].z <<"]" <<std::endl;
    }
    
    ROS_INFO("Creating Path  ...");
    CreatePath();
    ROS_INFO("Created Path : Size = %lu",traj.points.size());

}


void ReadPath::CreatePath()
{
    time_t ttime = time(0);
    tm *local_time = localtime(&ttime);
    
    string year_, month_, day_, hour_, min_, sec_;
    year_ = to_string(1900 + local_time->tm_year);
    month_ = to_string(1 + local_time->tm_mon);
    day_ = to_string(local_time->tm_mday);
    hour_ = to_string(local_time->tm_hour); 
    min_ = to_string(local_time->tm_min); 
    sec_ = to_string(1 + local_time->tm_sec);

	double d_to_interp_, d1_, d2_;
	int r_;
	bool interpol_;
	vector<geometry_msgs::Vector3> vec_pose_ugv, aux_v_pose_ugv;
	vector<geometry_msgs::Quaternion> vec_rot_ugv, aux_v_rot_ugv;
	geometry_msgs::Vector3 p_int_ugv_;

    vec_pose_ugv.clear(); aux_v_pose_ugv.clear();
	vec_rot_ugv.clear(); aux_v_rot_ugv.clear();

	// Interpolate vector
    int count = 0;
	for (int i=0 ; i < v_path.size()-1; i++){
		d_to_interp_ = 0.4;
		interpol_ = false;
		d1_ = sqrt ( pow(v_path[i+1].x - v_path[i].x,2) + 
                     pow(v_path[i+1].y - v_path[i].y,2) + 
                     pow(v_path[i+1].z - v_path[i].z,2) );
        geometry_msgs::Quaternion q_ = getOrientation(v_path[i+1], v_path[i]);

		if (d1_ > d_to_interp_ ){
			r_ = floor(d1_/d_to_interp_);
			interpol_ = true;
		}
		if (interpol_){
			for (int j=0 ; j < r_ ; j++){
				p_int_ugv_.x = v_path[i].x + (j)*(v_path[i+1].x - v_path[i].x)/(r_+1);
				p_int_ugv_.y = v_path[i].y + (j)*(v_path[i+1].y - v_path[i].y)/(r_+1);
				p_int_ugv_.z = v_path[i].z + (j)*(v_path[i+1].z - v_path[i].z)/(r_+1);
				aux_v_pose_ugv.push_back(p_int_ugv_);
				aux_v_rot_ugv.push_back(q_);
			}
		}else{
			aux_v_pose_ugv.push_back(v_path[i]);
			aux_v_rot_ugv.push_back(q_);
		}

        if (i == v_path.size()-2){
			aux_v_pose_ugv.push_back(v_path[i+1]);
			aux_v_rot_ugv.push_back(q_);
		}

	}

    int s_ = aux_v_pose_ugv.size()-1;
    for(int i = s_ ; i >=0  ; i--){
        vec_pose_ugv.push_back(aux_v_pose_ugv[i]);
        vec_rot_ugv.push_back(aux_v_rot_ugv[i]);
        std::cout<<"Interpolated path : ["<< i <<"] : ["<< vec_pose_ugv[s_-i].x  <<" , "<< vec_pose_ugv[s_-i].y <<" , "<< vec_pose_ugv[s_-i].z <<"]" <<std::endl;
        
    }
	
    if (use_create_file) {
        // Root of our file
        YAML::Node root_ugv;

        // Populate emitter
        YAML::Emitter emitter;

        // // Create a node listing some values
        root_ugv["ugv_path"] = YAML::Node(YAML::NodeType::Map);
        // // We now will write our values under root["MyNode"]
        YAML::Node node = root_ugv["ugv_path"];
        // YAML::Node node;
        // Write some values
        int size_ = vec_pose_ugv.size();

        node["header"] = "ugv_path";
        node["seq"] = 1;
        node["stamp"] = hour_+min_+sec_;
        node["frame_id"] = "ugv";
        node["size"] = size_;
        
        for(int i=0 ; i < vec_pose_ugv.size(); i++){
            node["poses"+to_string(i)]["header"] = "ugv"+to_string(i);
            node["poses"+to_string(i)]["seq"] = i;
            node["poses"+to_string(i)]["frame_id"] = "ugv";  
            node["poses"+to_string(i)]["pose"]["position"]["x"] = vec_pose_ugv[i].x;
            node["poses"+to_string(i)]["pose"]["position"]["y"] = vec_pose_ugv[i].y;
            node["poses"+to_string(i)]["pose"]["position"]["z"] = vec_pose_ugv[i].z;
            node["poses"+to_string(i)]["pose"]["orientation"]["x"] = vec_rot_ugv[i].x;
            node["poses"+to_string(i)]["pose"]["orientation"]["y"] = vec_rot_ugv[i].y;
            node["poses"+to_string(i)]["pose"]["orientation"]["z"] = vec_rot_ugv[i].z;
            node["poses"+to_string(i)]["pose"]["orientation"]["w"] = vec_rot_ugv[i].w;
        }

        emitter << root_ugv;

        // Write to file
        std::ofstream fout;
        fout.open(ros_node_name+year_+"_"+month_+"_"+day_+"_"+hour_+min_+sec_ +".yaml", std::ofstream::app);
        fout << emitter.c_str();

        std::cout << "Saved Path Optimized" << std::endl << std::endl;
    }
    else{
        trajectory.transforms.resize(1);
        trajectory.velocities.resize(1);
        trajectory.accelerations.resize(1);
        for (size_t i=0; i< vec_pose_ugv.size(); i++){
            trajectory.transforms[0].translation.x = vec_pose_ugv[i].x;
            trajectory.transforms[0].translation.y = vec_pose_ugv[i].y;
            trajectory.transforms[0].translation.z = vec_pose_ugv[i].z;
            trajectory.transforms[0].rotation.x = vec_rot_ugv[i].x;
            trajectory.transforms[0].rotation.y = vec_rot_ugv[i].y;
            trajectory.transforms[0].rotation.z = vec_rot_ugv[i].z;
            trajectory.transforms[0].rotation.w = vec_rot_ugv[i].w;
            trajectory.velocities[0].linear.x = 0.0;
            trajectory.velocities[0].linear.y = 0.0;
            trajectory.velocities[0].linear.z = 0.0;
            trajectory.accelerations[0].linear.x = 0.0;
            trajectory.accelerations[0].linear.y = 0.0;
            trajectory.accelerations[0].linear.z = 0.0;
            trajectory.time_from_start = ros::Duration(0.5);
            traj.header.stamp = ros::Time::now();
		    traj.points.push_back(trajectory);
        }
    }
}

geometry_msgs::Quaternion ReadPath::getOrientation(geometry_msgs::Vector3 p1_, geometry_msgs::Vector3 p2_)
{
	geometry_msgs::Quaternion _quat;
	tf::Quaternion _q;

	double yaw_ = atan2(p2_.y - p1_.y, p2_.x - p2_.x);
	_q.setRPY(0.0, 0.0, yaw_);

    _quat.x = _q.x();
	_quat.y = _q.y();
	_quat.z = _q.z();
	_quat.w = _q.w();

    return _quat;
}

void ReadPath::configServices()
{
    ROS_INFO("%s Node: Initialazing UGV Navigation Client", ros_node_name.c_str());
    NavigationClient.reset(new NavigateClient("/Navigation", true));
    NavigationClient->waitForServer();
    ROS_INFO("%s Node: Initialazed UGV Navigation Client", ros_node_name.c_str());
}

void ReadPath::ManagePath()
{

    int size_ = traj.points.size();

    ugv_goal3D.global_goal.position.x = traj.points.at(num_wp).transforms[0].translation.x;
    ugv_goal3D.global_goal.position.y = traj.points.at(num_wp).transforms[0].translation.y;
    ugv_goal3D.global_goal.position.z = traj.points.at(num_wp).transforms[0].translation.z;
    ugv_goal3D.global_goal.orientation.x = traj.points.at(num_wp).transforms[0].rotation.x;
    ugv_goal3D.global_goal.orientation.y = traj.points.at(num_wp).transforms[0].rotation.y;
    ugv_goal3D.global_goal.orientation.z = traj.points.at(num_wp).transforms[0].rotation.z;
    ugv_goal3D.global_goal.orientation.w = traj.points.at(num_wp).transforms[0].rotation.w;

    ros::Duration(0.2).sleep();  //DONT forget delete

    if(!sent_new_ugv_wp){
        ROS_INFO("Sending UGV WayPoint [%i/%i] to tracker: goal[%f %f %f]", num_wp, size_,
            ugv_goal3D.global_goal.position.x,
            ugv_goal3D.global_goal.position.y,
            ugv_goal3D.global_goal.position.z);
        NavigationClient->sendGoal(ugv_goal3D);
        ROS_INFO("Sent WayPoint for UGV ... Waiting for finishing maneuver");
        sent_new_ugv_wp = true;
        time_count_ugv = ros::Time::now();
    } 

        // TODO: wait also for the UGV client
        if ( NavigationClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            is_ugv_in_waypoint = true;
            ROS_INFO("UGV Path Tracker: Goal [%i/%i] Achieved",num_wp + 1,size_);

        }
        else if (NavigationClient->getState() == actionlib::SimpleClientGoalState::ABORTED) 
        {
              ROS_INFO("UGV Goal aborted by path tracker");
              return;
        }
        else if ( NavigationClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED) 
        { 
              ROS_INFO("UGV Goal preempted by path tracker");
              return;
        }

        if(is_ugv_in_waypoint) {
          std::cout <<" " << std::endl;
          ROS_INFO("\t\tSuccessfully achieved WayPoint [%i/%i]\n",num_wp + 1,size_);
          sent_new_ugv_wp = false;
          num_wp++;
        } else {
          //ROS_WARN("\t\tExecuting maneauvere to reach WayPoint [%i/%i]", num_wp + 1,size_);
        }

        if(ros::Time::now() - time_count_ugv > ros::Duration(time_max)) {
          std::cout <<" " << std::endl;
          ROS_ERROR("\t\tWasn't posible to reach UGV WayPoint [%i/%i] ", num_wp + 1,size_);
          NavigationClient->cancelAllGoals();
        }

        if (num_wp == size_){
          printf("\n\tWell Done !!!\n");
          printf("\n\tMission Finished: WPs achived\n");
          printf("\n\tInitializing flags !!\n");
        }
    ros::spinOnce();
}