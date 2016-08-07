#ifndef _image_convert_H_
#define _image_convert_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

class image_convert
{
    public:

         image_convert();
        virtual ~ image_convert(){};

        void run();

    private:
        /* ROS node */
        ros::NodeHandle nh;
        /* Message Publishers & Subscribers */
        image_transport::Subscriber m_imageSub;
        image_transport::Publisher m_imagePub;
        cv::Size m_size;

        void imageClbk ( const sensor_msgs::ImageConstPtr& msg);

};


#endif
