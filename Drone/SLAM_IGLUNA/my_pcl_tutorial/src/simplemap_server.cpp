/*
This is a test version of occupancy_map_generator"
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_simple.h"

#include <cmath>
#include <vector>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>

#include <nav_msgs/OccupancyGrid.h>
#include "yaml-cpp/yaml.h"
#include <fstream>

bool simple(my_pcl_tutorial::occupancymap_simple::Request  &req,
               my_pcl_tutorial::occupancymap_simple::Response &res)
{

// Initialization

ROS_INFO("request: file_in=%s, file_out=%s, frame_id=%s, resolution=%lf", req.file_in.c_str(), req.file_out.c_str(), req.frame_id.c_str(), req.resolution);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read (req.file_in, *cloud);
int width;
int height;
double origin_x;
double origin_y;
double resolution=req.resolution;

//Get Min and Max x and y and compute width and height of the 2D map
float min_x = cloud->points[0].x, min_y = cloud->points[0].y, max_x = cloud->points[0].x, max_y = cloud->points[0].y,min_z=cloud->points[0].z,max_z=cloud->points[0].z;
for (size_t i = 1; i < cloud->points.size (); ++i){
       if(cloud->points[i].x <= min_x )
           min_x = cloud->points[i].x;
       if(cloud->points[i].y <= min_y )
           min_y = cloud->points[i].y;
       if(cloud->points[i].z <= min_z )
           min_z = cloud->points[i].z;
       if(cloud->points[i].x >= max_x )
           max_x = cloud->points[i].x;
        if(cloud->points[i].y >= max_y )
           max_y = cloud->points[i].y;
        if(cloud->points[i].z >= max_z )
           max_z = cloud->points[i].z;
   }
origin_x=min_x;
origin_y=min_y;
//Dimensions in meter
float width_float  = max_x - min_x;
float height_float = max_y - min_y;
//Number of cells necessary
float width_scaled = width_float  / resolution - 1/2;
float height_scaled= height_float / resolution - 1/2;
width  = int(width_scaled)+2;
height = int(height_scaled)+2; //Truncation to the lower int means we need to add 1 cell, and we want origin point to fall in the middle of the cell so we need to add 1 another cell. 
std::cout<< "\n Information about the grid map:\n";
std::cout<< "width=";
std::cout<< width; 
std::cout<<" height="; 
std::cout<<height;std::cout<< " origine_x= "; std::cout<<min_x;std::cout << " max_x="; std::cout<< max_x;std::cout<< " max_y= "; std::cout << max_y; std::cout<<" origine_y=";std::cout<<min_y;std::cout<< " min_z=" ;std::cout<<min_z; std::cout << " max_z="; std::cout << max_z;

//Instantiate 2D map
std::vector<int8_t> occupancygridlist(width*height);
for (int i=0;i!=occupancygridlist.size();i++){
occupancygridlist[i]=0;}


//iteration on the point cloud to fill the 2D map

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
pcl::PointXYZ search_point;
search_point.x=it->x;
search_point.y=it->y;
search_point.z=it->z;
int i,j;
//You need to read the grid as the "abscisse/ordonnée" graph-math convention where a point A(i,j) have i on the horizontal axis and the j value read on the vertical axis and NOT AS A MATRIX ROW/COLUMN CONVENTION !
//abscisse
float cell_x= (search_point.x-origin_x)/resolution -1/2; //normalement compris entre i-1 et i
if (cell_x<=0) i=0;
else i= int(cell_x)+1;
//Ordonnée
float cell_y= (search_point.y-origin_y)/resolution -1/2; //normalement compris entre j-1 et j
if (cell_y<=0) j=0;
else j= int(cell_y)+1;
occupancygridlist[j*width+i]=100;
}


nav_msgs::OccupancyGrid occupancygrid;
occupancygrid.header.frame_id=req.frame_id;
occupancygrid.info.resolution=resolution;
occupancygrid.info.width=width;
occupancygrid.info.height=height;
occupancygrid.info.origin.position.x=origin_x;
occupancygrid.info.origin.position.y=origin_y;
occupancygrid.info.origin.orientation.x = 0.0;
occupancygrid.info.origin.orientation.y = 0.0;
occupancygrid.info.origin.orientation.z = 0.0;
occupancygrid.info.origin.orientation.w = 1.0;
occupancygrid.data=occupancygridlist;


//Save the map to a file
std::string mapname=req.file_out;
int threshold_free=25;
int threshold_occupied=65;


//As a pgm file 

std::string mapdatafile = mapname + "custom.pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_INFO( "Couldn't save map file to %s", mapdatafile.c_str());
        return 0;
      }

      fprintf(out, "P5\n# CREATOR: Grid_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              occupancygrid.info.resolution, occupancygrid.info.width, occupancygrid.info.height);
        for(unsigned int i=0;i!=occupancygrid.info.height;i++){
	for(unsigned int j=0;j!=occupancygrid.info.width;j++){
	unsigned int x=j+(occupancygrid.info.height-1-i)*occupancygrid.info.width;
          if (occupancygrid.data[x] >= 0 && occupancygrid.data[x] <= threshold_free) { // Free is 254 : white
            fputc(254, out);
          } else if (occupancygrid.data[x] >= threshold_occupied) { // Occupied is black : 000
            fputc(000, out);
          } else { //unknown is 205 gray scale
            fputc(205, out);
          }
        }}
      fclose(out);

//As a txt file 
std::string maptxtdatafile = mapname + "custom.txt";
 ROS_INFO("Writing map occupancy data to %s", maptxtdatafile.c_str());
      FILE* txt = fopen(maptxtdatafile.c_str(), "w");
      if (!txt)
      {
        std::cout<< "Couldn't save map file to %s", maptxtdatafile.c_str();
        return 0;
      }

      fprintf(txt, "%.3f\n%d\n%d\n%s\n%.3f\n%.3f\n", occupancygrid.info.resolution, occupancygrid.info.width, occupancygrid.info.height, occupancygrid.header.frame_id.c_str(),occupancygrid.info.origin.position.x, occupancygrid.info.origin.position.y);
      for(unsigned int cell = 0; cell < occupancygrid.data.size(); cell++) {
       fprintf(txt, "%d ", occupancygrid.data[cell]);
      }
      fclose(txt);

   ROS_INFO("Sending back response:\n 2D Occupancy grid map generation from 3D DEM map was a success \n Some information about the occupancy grid map : \n Coordinates of origin point :\n Origin.x = %lf \n Origin.y=%lf \n Occupancy width=%d \n Occupancy height=%d \n Resolution=%lfm \n", origin_x, origin_y, width, height, resolution) ;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_simple_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("occupancymap_simple", simple);
  ROS_INFO("Ready to generate a 2D Occupancy Grid map from a 3D DEM map (Pointcloud .pcd type)");
  ros::spin();

  return 0;
}
