#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include "pcl_ros/point_cloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <std_srvs/Empty.h>
#include <string.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <dynamic_reconfigure/server.h>
#include <lidar_object_detection/parametersConfig.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>("file_" + std::string(argv[1]) + ".pcd", *input_cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file. \n");
        return (-1);
    }

    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(input_cloud);
    // while (!viewer.wasStopped())
    // {
    // }

    pcl::PassThrough<pcl::PointXYZI> x_pass, y_pass, i_pass;

    x_pass.setInputCloud(input_cloud);
    x_pass.setFilterFieldName("x");
    x_pass.setFilterLimits(-3.0, 3.0);
    x_pass.filter(*input_cloud);

    y_pass.setInputCloud(input_cloud);
    y_pass.setFilterFieldName("y");
    y_pass.setFilterLimits(-3.0, 3.0);
    y_pass.filter(*input_cloud);

    i_pass.setInputCloud(input_cloud);
    i_pass.setFilterFieldName("intensity");
    i_pass.setFilterLimits(160, 3000);
    i_pass.filter(*input_cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(10.0);
    sor.setStddevMulThresh(0.1);
    sor.filter(*input_cloud);

    pcl::visualization::CloudViewer viewer1("Simple Cloud Viewer");
    viewer1.showCloud(input_cloud);
    while (!viewer1.wasStopped())
    {
    }

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(input_cloud);
    ne.setKSearch(40);
    ne.compute(*cloud_normals);

    // processed cloud formation
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    filtered_cloud->width = input_cloud->width;
    filtered_cloud->height = input_cloud->height;
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->points.resize(filtered_cloud->width * filtered_cloud->height);

    // plane fitting
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.05);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(input_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment (*inliers, *coefficients_plane);

    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_CYLINDER);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setNormalDistanceWeight(0.02);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.01);
    // seg.setRadiusLimits(0.0, 0.05);
    // seg.setInputCloud(input_cloud);
    // seg.setInputNormals(cloud_normals);
    // seg.segment (*inliers, *coefficients_plane);
    for (const auto &idx : inliers->indices)
        filtered_cloud->points[idx] = input_cloud->points[idx];
        

    pcl::visualization::CloudViewer viewer2("Simple Cloud Viewer");
    viewer2.showCloud(filtered_cloud);
    while (!viewer2.wasStopped())
    {
    }
}