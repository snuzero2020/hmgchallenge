#include "cluster_class.h"
#include <iostream>

using namespace std;

ClusterExtraction::ClusterExtraction(bool use_voxelgrid, std::string temp_data_path){
    // Read point cloud data
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read(temp_data_path, *cloud);
    std::cout << "PointCloud before filtering has " << cloud->size() << "data points." << std::endl;

    this->use_voxelgrid = use_voxelgrid;
    this->temp_data_path = temp_data_path;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_original_cloud(){
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_vg_cloud(){
    return vg_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_cloud_plane(){
    return cloud_plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_plane_removed(){
    return plane_removed;
}

std::vector<pcl::PointIndices> ClusterExtraction::return_cluster_indices(){
    return cluster_indices;
}

void ClusterExtraction::voxelgrid_filter(){
    float leaf_size;
    ros::param::get("/leaf_size", leaf_size);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*vg_cloud);
    std::cout << "PointCloud after voxelgrid_filter has: " << vg_cloud->size ()  << " data points." << std::endl;
}

void ClusterExtraction::ransac_plane_remove(){
    int max_iterations;
    double distance_threshold;
    ros::param::get("/max_iterations", max_iterations);
    ros::param::get("/distance_threshold", distance_threshold);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    cloud_plane = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    plane_removed = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iterations);
    seg.setDistanceThreshold(distance_threshold);

    if(use_voxelgrid)
        seg.setInputCloud(vg_cloud);
    else
        seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planer model for the given dataset." << std::endl;
    }
    else{
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        
        if(use_voxelgrid)
            extract.setInputCloud(vg_cloud);
        else
            extract.setInputCloud(cloud);
        
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        extract.setNegative(true);
        extract.filter(*plane_removed);
        std::cout << "PointCloud after plane ransac_palne_remove has: " << plane_removed->size ()  << " data points." << std::endl;
    }
}

void ClusterExtraction::euclidean_cluster(){
    double cluster_tolerance;
    int max_cluster_size, min_cluster_size;
    ros::param::get("/cluster_tolerance", cluster_tolerance);
    ros::param::get("/max_cluster_size", max_cluster_size);
    ros::param::get("/min_cluster_size", min_cluster_size);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(plane_removed);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(plane_removed);
    ec.extract(cluster_indices);
}