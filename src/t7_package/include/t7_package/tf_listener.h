#ifndef _TF_LISTENER_H_
#define _TF_LISTENER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>


class Listener{
public:

	Listener();
	~Listener(){};

	void run();

private:

	ros::NodeHandle node;
	tf::TransformListener listenerr;
	//ros::ServiceServer setFrame;
	std::string frame;
	float height;
	//void SetFrame(t7_package::setFrame::Request&   req, t7_package::setFrame::Response&  res);

};
#endif
