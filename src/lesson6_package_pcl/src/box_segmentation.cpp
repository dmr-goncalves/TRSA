#include "ros/ros.h"
#include "/home/duarteg/trsa_ws/src/lesson6_package_pcl/include/lesson6_package_pcl/box_segmentation.h"


Box_Segmentation::Box_Segmentation(ros::NodeHandle n) : n_(n)
{
	cloud_sub_ = n_.subscribe("/octomap_cloud", 1000, &Box_Segmentation::cloudCallback, this);
	treated_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/box_cloud",1);
}

void Box_Segmentation::run(){
	ros::spin();
}
void Box_Segmentation::cloudCallback (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	//converting the sensor_msgs::PointCloud2 into pcl::PointCloud type
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::fromROSMsg(*cloud_in, *pcl_cloud);

	//restrain the region of interest to a small area right in front of the laser scanner, a
	//pcl::PassThrough filter is applied to eliminate all points farther than 0.5 m distance from the sensor
	//in the y-axis
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-1.2, 0.5);
	pass.setInputCloud(pcl_cloud);
	pass.filter(*pcl_cloud);

	//model fitting process to a plane perpendicular to the x-axis
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.08); //0.03
	seg.setAxis (Eigen::Vector3f(1, 0, 0));
	seg.setEpsAngle (0.04); //0.02
	seg.setInputCloud (pcl_cloud);
	seg.segment (*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr treated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	for (std::vector<int>::const_iterator it = inliers->indices.begin(); it != inliers->indices.end (); ++it)
	treated_cloud->push_back(pcl_cloud->points[*it]);

	//radial outlier filter is applied to remove any points linking the two boxes’ front side planes
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> radialFilter;
	radialFilter.setInputCloud(treated_cloud);
	radialFilter.setRadiusSearch(0.03);
	radialFilter.setMinNeighborsInRadius (20);
	radialFilter.filter (*treated_cloud);
	//clustering algorithm based on the Euclidean distance between the points is applied, to
	//distinguish the two boxe
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud (treated_cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02);
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (10000);
	ec.setSearchMethod (kdtree);
	ec.setInputCloud (treated_cloud);
	ec.extract (cluster_indices);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::PointXYZI cluster_point;
	double cluster_final_average;
	int cluster_id=0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end ();
	++it, ++cluster_id)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
		{
			cluster_point.x = treated_cloud->points[*pit].x;
			cluster_point.y = treated_cloud->points[*pit].y;
			cluster_point.z = treated_cloud->points[*pit].z;
			cluster_point.intensity = cluster_id;
			cluster_cloud->push_back(cluster_point);
		}
	}

	//callback function is then finallized by setting up the sensor_msgs::PointCloud2 message with the
	//clustered cloud information, and publishing it.

	sensor_msgs::PointCloud2 cloud;
	pcl::toROSMsg(*cluster_cloud, cloud);
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = cloud_in->header.frame_id;
	treated_cloud_pub_.publish(cloud);

}
