#ifndef _TF_BROADCASTER_H_
#define _TF_BROADCASTER_H_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

class Broadcaster{
    public:

        Broadcaster();
        ~Broadcaster(){};

        void run();

    private:

        ros::NodeHandle node;
		ros::Subscriber sub;
		ros::Publisher pub;
		float height;
		void callback(const std_msgs::Float32ConstPtr& msg);

};
#endif
