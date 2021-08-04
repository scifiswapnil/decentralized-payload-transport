#include <detection.h>

ros::Publisher pub;

int i = 0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloudPtr);
  pass.setFilterLimitsNegative (false);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-3.0, 3.0);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3.0, 3.0);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits(120, 3000);
  pass.filter (cloud_filtered);
  
  pcl_conversions::fromPCL(cloud_filtered, output);
  pcl_conversions::toPCL(output, *cloud);
  
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setMeanK(100);
  sor.setStddevMulThresh(0.02);
  sor.filter(cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  
  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}