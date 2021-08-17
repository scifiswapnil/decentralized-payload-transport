#include <detection.h>

double x_positive = +1.0;
double x_negative = -1.0;
double y_positive = +1.0;
double y_negative = -1.0;
double min_intensity = 150;
double max_intensity = 3000;
double sor_meanK = 10.0;
double sor_StddevMulThresh = 1.0;
double normal_esitmator_KSearch = 50.0;
double SACDistanceThreshold = 0.07;

ros::Publisher primary_filter_publisher, processed_plane_cloud_publisher, processed_bot_cloud_publisher;
ros::Publisher pallet_marker_publisher, bot_marker_pub, plane_marker_pub;
ros::Publisher pallet_pose, agent_pose;
pcl::PointCloud<pcl::PointXYZI>::Ptr save_cloud(new pcl::PointCloud<pcl::PointXYZI>);
int file_pcd_count = 0;

visualization_msgs::Marker create_marker(pcl::PointXYZI center)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
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

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;

  // Estimate point normals
  ne.setSearchMethod(tree);
  ne.setInputCloud(input_cloud);
  ne.setKSearch(normal_esitmator_KSearch);
  ne.compute(*cloud_normals);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.05);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(SACDistanceThreshold);
  seg.setInputCloud(input_cloud);
  seg.setInputNormals(cloud_normals);
  seg.segment(*inliers, *coefficients_plane);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR("No planar model found. Relax thresholds and continue.\n");
    return;
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract, extract_neg;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*filtered_cloud);

  // processed cloud formation
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_bot(new pcl::PointCloud<pcl::PointXYZI>);
  filtered_cloud_bot->width = input_cloud->width;
  filtered_cloud_bot->height = input_cloud->height;
  filtered_cloud_bot->header = input_cloud->header;
  filtered_cloud_bot->points.resize(filtered_cloud_bot->width * filtered_cloud_bot->height);

  extract_neg.setInputCloud(input_cloud);
  extract_neg.setIndices(inliers);
  extract_neg.setNegative(true);
  extract_neg.filter(*filtered_cloud_bot);

  pcl::PointXYZI center, center1;
  pcl::CentroidPoint<pcl::PointXYZI> centroid, centroid1;

  for (int j = 0; j < filtered_cloud->points.size(); j++)
  {
    auto point = filtered_cloud->points[j];
    if (point.x != 0.0 && point.y != 0.0 && point.z != 0.0)
      centroid.add(point);
  }
  centroid.get(center);

  for (int j = 0; j < filtered_cloud_bot->points.size(); j++)
  {
    auto point = filtered_cloud_bot->points[j];
    if (point.x != 0.0 && point.y != 0.0 && point.z != 0.0)
      centroid1.add(point);
  }
  centroid1.get(center1);

  pcl::toROSMsg(*filtered_cloud, output);
  processed_plane_cloud_publisher.publish(output);

  pcl::toROSMsg(*filtered_cloud_bot, output);
  processed_bot_cloud_publisher.publish(output);

  //Moment of Inertia
  pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
  feature_extractor.setInputCloud(filtered_cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;

  pcl::PointXYZI min_point_OBB;
  pcl::PointXYZI max_point_OBB;
  pcl::PointXYZI position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues(major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter(mass_center);

  Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat(rotational_matrix_OBB);

  if (center1.x != 0.0 && center1.y != 0.0 && center1.z != 0.0)
  {
    geometry_msgs::PoseStamped APose;
    APose.header.stamp = ros::Time::now();
    APose.header.frame_id = "velodyne";
    APose.pose.position.x = center1.x;
    APose.pose.position.y = center1.y;
    APose.pose.position.z = center1.z;
    APose.pose.orientation.x = 0.0;
    APose.pose.orientation.y = 0.0;
    APose.pose.orientation.z = 0.0;
    APose.pose.orientation.w = 1.0;
    agent_pose.publish(APose);
    bot_marker_pub.publish(create_marker(center1));
  }

  if (center.x != 0.0 && center.y != 0.0 && center.z != 0.0)
  {
    geometry_msgs::PoseStamped PPose;
    PPose.header.stamp = ros::Time::now();
    PPose.header.frame_id = "velodyne";
    PPose.pose.position.x = position_OBB.x;
    PPose.pose.position.y = position_OBB.y;
    PPose.pose.position.z = position_OBB.z;
    PPose.pose.orientation.x = quat.x();
    PPose.pose.orientation.y = quat.y();
    PPose.pose.orientation.z = quat.z();
    PPose.pose.orientation.w = quat.w();
    pallet_pose.publish(PPose);
    pallet_marker_publisher.publish(create_marker(center));
  }

  //Visualisation Marker
  std::string ns;
  float r;
  float g;
  float b;
  visualization_msgs::MarkerArray msg_marker;
  visualization_msgs::Marker bbx_marker;
  bbx_marker.header.frame_id = "velodyne";
  bbx_marker.header.stamp = ros::Time::now();
  bbx_marker.ns = ns;
  bbx_marker.type = visualization_msgs::Marker::CUBE;
  bbx_marker.action = visualization_msgs::Marker::ADD;
  bbx_marker.pose.position.x = position_OBB.x;
  bbx_marker.pose.position.y = position_OBB.y;
  bbx_marker.pose.position.z = position_OBB.z;
  bbx_marker.pose.orientation.x = quat.x();
  bbx_marker.pose.orientation.y = quat.y();
  bbx_marker.pose.orientation.z = quat.z();
  bbx_marker.pose.orientation.w = quat.w();
  bbx_marker.scale.x = (max_point_OBB.x - min_point_OBB.x);
  bbx_marker.scale.y = (max_point_OBB.y - min_point_OBB.y);
  bbx_marker.scale.z = (max_point_OBB.z - min_point_OBB.z);
  bbx_marker.color.b = b;
  bbx_marker.color.g = g;
  bbx_marker.color.r = r;
  bbx_marker.color.a = 0.4;
  bbx_marker.lifetime = ros::Duration();
  msg_marker.markers.push_back(bbx_marker);
  plane_marker_pub.publish(msg_marker);
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

  processed_plane_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("plane_object", 1);
  processed_bot_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("agent_object", 1);
  primary_filter_publisher = nh.advertise<sensor_msgs::PointCloud2>("primary_output", 1);

  pallet_marker_publisher = nh.advertise<visualization_msgs::Marker>("pallet_centroid", 1);
  bot_marker_pub = nh.advertise<visualization_msgs::Marker>("other_agent", 1);
  plane_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("plane_marker", 1);

  pallet_pose = nh.advertise<geometry_msgs::PoseStamped>("pallet_pose", 1);
  agent_pose = nh.advertise<geometry_msgs::PoseStamped>("agent_pose", 1);

  ros::ServiceServer service = nh.advertiseService("save_point_cloud", save);

  // Spin
  ros::spin();
}