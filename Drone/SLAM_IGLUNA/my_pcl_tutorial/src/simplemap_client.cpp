/*
This is a test version of occupancy_map_generator"
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_simple.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_simple");
  if (argc != 5)
  {
    ROS_INFO("usage: occupancymap_simple file_in file_out frame_id resolution resolution_discretized resolution_grid_map_pcl_node");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_simple>("occupancymap_simple");
  my_pcl_tutorial::occupancymap_simple srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  srv.request.frame_id=argv[3];
  srv.request.resolution = atof(argv[4]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map simple was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map simple");
    return 1;
  }

  return 0;
}





