#include "t7_package/tf_broadcaster.h"


Broadcaster::Broadcaster(){

		sub = node.subscribe("/tripod/height", 10, &Broadcaster::callback, this);

}

void Broadcaster::callback(const std_msgs::Float32ConstPtr& msg){

   height = msg->data;
   static tf::TransformBroadcaster br;
   tf::Transform transform;
   transform.setOrigin( tf::Vector3(0.0, 0.0,  height) );
   tf::Quaternion q;
   q.setRPY(0, 0, 0);
   transform.setRotation(q);
   br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "sensor_frame"));
}
 void Broadcaster::run()
 {
     ros::spin();
 }
