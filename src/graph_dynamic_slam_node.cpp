#include <ros/ros.h>
#include "pointcloud_matching.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_dynamic_slam_node");
  ros::NodeHandle n;

  graph_dynamic_slam::PointcloudMatching pcMatch;

  ros::spin();
}
