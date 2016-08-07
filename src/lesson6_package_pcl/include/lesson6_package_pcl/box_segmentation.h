#ifndef _box_segmentation_H_
#define _box_segmentatio_H_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class Box_Segmentation{
public:

  Box_Segmentation(ros::NodeHandle n);
  ~Box_Segmentation(){};
  void run();

private:
ros::NodeHandle n_;
ros::Subscriber cloud_sub_;
ros::Publisher treated_cloud_pub_;
 
void cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in);

};
#endif
