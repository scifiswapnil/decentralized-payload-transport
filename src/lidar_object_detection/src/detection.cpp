#include <detection.h>

double x_positive = +1.0;
double x_negative = -1.0;
double y_positive = +1.0;
double y_negative = -1.0;
double min_intensity = 150;
double max_intensity = 3000;
double sor_meanK = 10.0;
double sor_StddevMulThresh = 1.0;
double normal_esitmator_KSearch = 100.0;
double ClusterMinSize = 80.0;
double ClusterMaxSize = 1000000.0;
double ClusterNumberOfNeighbours = 20.0;
double SACDistanceThreshold = 0.03;

ros::Publisher processed_cloud_publisher, primary_filter_publisher, marker_publisher;
pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZI>);
int file_pcd_count = 0;

visualization_msgs::Marker create_marker(pcl::PointXYZI center)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center.x;
  marker.pose.position.y = center.y;
  marker.pose.position.z = center.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  return marker;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  sensor_msgs::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*cloud_msg, *input_cloud);
  pcl::fromROSMsg(*cloud_msg, *save_cloud);

  pcl::PassThrough<pcl::PointXYZI> x_pass, y_pass, i_pass;
  try
  {
    x_pass.setInputCloud(input_cloud);
    x_pass.setFilterFieldName("x");
    x_pass.setFilterLimits(x_negative, x_positive);
    x_pass.filter(*input_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    std::cerr << "x filter failed" << std::endl;
  }

  try
  {
    y_pass.setInputCloud(input_cloud);
    y_pass.setFilterFieldName("y");
    y_pass.setFilterLimits(y_negative, y_positive);
    y_pass.filter(*input_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    std::cerr << "y filter failed" << std::endl;
  }

  try
  {
    i_pass.setInputCloud(input_cloud);
    i_pass.setFilterFieldName("intensity");
    i_pass.setFilterLimits(min_intensity, max_intensity);
    i_pass.filter(*input_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    std::cerr << "intensity filter failed" << std::endl;
  }

  try
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(sor_meanK);
    sor.setStddevMulThresh(sor_StddevMulThresh);
    sor.filter(*input_cloud);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    std::cerr << "sor filter failed" << std::endl;
  }

  pcl::toROSMsg(*input_cloud, output);
  primary_filter_publisher.publish(output);

  // processed cloud formation
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  filtered_cloud->width = input_cloud->width;
  filtered_cloud->height = input_cloud->height;
  filtered_cloud->header = input_cloud->header;
  filtered_cloud->points.resize(filtered_cloud->width * filtered_cloud->height);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(input_cloud);
  ne.setKSearch(40);
  ne.compute(*cloud_normals);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.05);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(input_cloud);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, *coefficients_plane);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("No planar model found. Relax thresholds and continue.\n");
    return;
  }

  for (const auto &idx : inliers->indices)
    filtered_cloud->points[idx] = input_cloud->points[idx];

  // pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
  // reg.setMinClusterSize(ClusterMinSize);
  // reg.setMaxClusterSize(ClusterMaxSize);
  // reg.setSearchMethod(tree);
  // reg.setNumberOfNeighbours(ClusterNumberOfNeighbours);
  // reg.setInputCloud(filtered_cloud);
  // reg.setInputNormals(cloud_normals);
  // reg.setSmoothnessThreshold(1.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold(1.0);
  // std::vector<pcl::PointIndices> clusters;
  // reg.extract(clusters);

  // // processed cloud formation
  // pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
  // filtered->width = input_cloud->width;
  // filtered->height = input_cloud->height;
  // filtered->header = input_cloud->header;
  // filtered->points.resize(filtered->width * filtered->height);

  // for (const auto &idx : clusters[0].indices)
  //   filtered->points[idx] = input_cloud->points[idx];

  pcl::PointXYZI center;
  pcl::CentroidPoint<pcl::PointXYZI> centroid;
  for (int j = 0; j < filtered_cloud->points.size(); j++)
  {
    auto point = filtered_cloud->points[j];
    if (point.x != 0.0 && point.y != 0.0 && point.z != 0.0)
      centroid.add(point);
  }
  centroid.get(center);
  marker_publisher.publish(create_marker(center));

  pcl::toROSMsg(*filtered_cloud, output);
  processed_cloud_publisher.publish(output);
}

void callback(lidar_object_detection::parametersConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request Received");
  x_positive = config.positive_x_ve;
  x_negative = config.negative_x_ve;
  y_positive = config.positive_y_ve;
  y_negative = config.negative_y_ve;
  min_intensity = config.min_intensity;
  max_intensity = config.max_intensity;
  sor_StddevMulThresh = config.sor_StddevMulThresh;
  sor_meanK = config.sor_meanK;
  normal_esitmator_KSearch = config.normal_esitmator_KSearch;
  ClusterMinSize = config.ClusterMinSize;
  ClusterMaxSize = config.ClusterMaxSize;
  ClusterNumberOfNeighbours = config.ClusterNumberOfNeighbours;
  SACDistanceThreshold = config.SACDistanceThreshold;
}

bool save(std_srvs::Empty::Request &req,
          std_srvs::Empty::Response &res)
{
  file_pcd_count++;
  pcl::io::savePCDFileASCII("file_" + std::to_string(file_pcd_count) + ".pcd", *save_cloud);
  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "lidar_object_detection");
  ros::NodeHandle nh;

  dynamic_reconfigure::Server<lidar_object_detection::parametersConfig> server;
  dynamic_reconfigure::Server<lidar_object_detection::parametersConfig>::CallbackType ddy_function;
  ddy_function = boost::bind(&callback, _1, _2);
  server.setCallback(ddy_function);

  ros::Subscriber input_cloud_subscriber = nh.subscribe("/velodyne_points", 1, cloud_cb);

  processed_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  primary_filter_publisher = nh.advertise<sensor_msgs::PointCloud2>("primary_output", 1);
  marker_publisher = nh.advertise<visualization_msgs::Marker>("pallet_centroid", 1);

  ros::ServiceServer service = nh.advertiseService("save_point_cloud", save);

  // Spin
  ros::spin();
}