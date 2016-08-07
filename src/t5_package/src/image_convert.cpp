#include <ros/ros.h>
#include "t5_package/image_convert.h"

image_convert::image_convert()
{
  image_transport::ImageTransport it(nh);
  m_imageSub = it.subscribe("/camera/image_raw", 1, &image_convert::imageClbk, this);
  m_imagePub = it.advertise("/camera/image_processed", 1);
}

void image_convert::run()
{
  ros::spin();
}

void image_convert::imageClbk ( const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cvImagePtr;
  cv::Mat grayImage, grayImageGauss;
  cv_bridge::CvImage out_msg;
  sensor_msgs::Image ros_image;

  try
  {
    // toCvCopy method copies the image data and returns a mutable CvImage
    cvImagePtr = cv_bridge::toCvCopy( msg );
    // toCvShare method shares the image data, returning a const CvImage
    // cv_bridge::CvImageConstPtr cvImagePtr = cv_bridge::toCvShare( rosImage );
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR( "cv_bridge exception: %s", e.what() );
  }

  //let's process the image using the pointer!
  cv::cvtColor(cvImagePtr->image, grayImage, CV_BGR2GRAY);
  //cv::cvtColor(cvImagePtr->image, cvImagePtr->image, CV_GRAY2BGR);
  cv::GaussianBlur(grayImage, grayImageGauss, cv::Size(21,21), 0, 0);


  out_msg.header.stamp = ros::Time::now() ; // timestamp from the system clock
  out_msg.header.frame_id = "/camera_link"; // camera position in the world
  out_msg.encoding = sensor_msgs::image_encodings::MONO8; // or other types
  out_msg.image = grayImageGauss; // your cv::Mat
  out_msg.toImageMsg( ros_image );
  // now we can publish the image…
  m_imagePub.publish(ros_image); // image_transport::Publisher

}
