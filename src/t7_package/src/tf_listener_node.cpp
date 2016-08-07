#include <ros/ros.h>
#include "t7_package/tf_listener.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "tf_listener_node");
  Listener listen;
  listen.run();
  return EXIT_SUCCESS;

};
