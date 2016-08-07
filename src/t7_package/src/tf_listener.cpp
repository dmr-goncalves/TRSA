#include <ros/ros.h>
#include "t7_package/tf_listener.h"


Listener::Listener(){

	/*setFrame = node.advertiseService("/setFrame", &Listener::SetFrame, this);
	frame = "base_footprint";*/

}


void Listener::run()
{

	ros::Rate rate(10.0);
	while (node.ok()){
		tf::StampedTransform transform;
		try{
			listenerr.lookupTransform("map", "base_footprint", ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		ROS_INFO("%.2f", transform.getOrigin().z());
		//ROS_INFO("%.2f",  sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().z(), 2)));
		rate.sleep();
	}
	ros::spin();

}
/*
void Listener::SetFrame(t7_package::setFrame::Request&   req, t7_package::setFrame::Response&  res){

	frame = req.Frame;
	return res.newFrame = frame;

}*/
