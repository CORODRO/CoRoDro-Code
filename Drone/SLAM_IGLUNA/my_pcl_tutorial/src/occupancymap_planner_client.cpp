/*
This ROS service uses a 2D grid map (generated by "occupancy_generator" ROS service) and a database of detected objects (generated by "ar_tracker_saver.cpp")
to create the HDDL problem file used by the TASK planner algorithm.
This is an old version. The latest version is on the ROVER_ENDING_VERSION_25_06.
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_planner.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_planner");
  if (argc != 7)
  {
    ROS_INFO("usage: occupancymap_planner file_in_map file_in_artag file_out resolution_discretized threshold_occupied threshold_unknown");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_planner>("occupancymap_planner");
  my_pcl_tutorial::occupancymap_planner srv;
  srv.request.file_in_map = argv[1];
  srv.request.file_in_artag = argv[2];
  srv.request.file_out = argv[3];
  srv.request.resolution_discretized = atof(argv[4]);
  srv.request.thr_occ = atof(argv[5]);
  srv.request.thr_unknown = atof(argv[6]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map planner was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map planner");
    return 1;
  }

  return 0;
}
