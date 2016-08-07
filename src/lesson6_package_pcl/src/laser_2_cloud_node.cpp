#include "ros/ros.h"
#include "/home/duarteg/trsa_ws/src/lesson6_package_pcl/include/lesson6_package_pcl/laser_2_cloud.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LaserScanToPointCloud");
  ros::NodeHandle n;
  LaserScan_2_PointCloud ls2pc(n);
  ls2pc.run();

  return 0;
}
