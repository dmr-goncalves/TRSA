#include "ros/ros.h"
#include "/home/duarteg/trsa_ws/src/lesson6_package_pcl/include/lesson6_package_pcl/box_segmentation.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "BoxSegmentation");
  ros::NodeHandle n;
  Box_Segmentation box_seg(n);
  box_seg.run();
  return 0;
}
