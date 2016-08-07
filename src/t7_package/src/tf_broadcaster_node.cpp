#include <ros/ros.h>
#include "t7_package/tf_broadcaster.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "tf_broadcaster_node");
  Broadcaster broadcast;
  broadcast.run();
  return EXIT_SUCCESS;

};
