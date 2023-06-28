#include "ugv_path_tracker/read_path.hpp"

int main (int argc, char** argv)
{
  std::string node_name = "read_path_node";
  
  ros::init (argc, argv, node_name);

  std::cout << "Initializing node: " << node_name << std::endl;

  ReadPath rp(node_name);
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    ros::spinOnce();
    if(rp.traj.points.size()  > 1)
      rp.ManagePath();
      loop_rate.sleep();
    }
	
  return 0;
}
