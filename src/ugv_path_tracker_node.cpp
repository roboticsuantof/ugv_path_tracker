#include <ros/ros.h>
#include <ugv_path_tracker/ugv_path_tracker.hpp>

int main (int argc, char** argv)
{
  ros::init (argc, argv, "ugv_traj_tracker_node");

  UGVPathTracker pt;
  pt.traj.points.clear();
  ros::Rate loop_rate(10);

  while (ros::ok())
    {
        ros::spinOnce();
        pt.navigate();
        loop_rate.sleep();
    }
	
  return 0;
}
