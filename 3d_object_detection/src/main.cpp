#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

bool use_voxelgrid;
std::string temp_data_path;

class ClusterExtraction
{
    private:
        ros::NodeHandle nh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr vg_cloud;

    public:
        ClusterExtraction(){
            // Read point cloud data
            cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCDReader reader;
            reader.read(temp_data_path, *cloud);
            std::cout << "PointCloud before filtering has " << cloud->size() << "data points." << std::endl;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr return_original_cloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr return_vg_cloud();
        void voxelgrid_filter();
};

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_original_cloud(){
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ClusterExtraction::return_vg_cloud(){
    return vg_cloud;
}

void ClusterExtraction::voxelgrid_filter(){
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*vg_cloud);
    std::cout << "PointCloud after voxel grid filtering has: " << vg_cloud->size ()  << " data points." << std::endl;
}

void cloud_visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::visualization::CloudViewer viewer("Point Cloud viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ()){}
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_detection_main");

    int visualize_cloud;
    ros::param::get("/visualize_cloud", visualize_cloud);
    ros::param::get("/use_voxelgrid", use_voxelgrid);
    ros::param::get("/temp_data_path", temp_data_path);

    clock_t startTime = clock();
    
    ClusterExtraction cluster_extraction;

    if(use_voxelgrid)
        cluster_extraction.voxelgrid_filter();

    std::cout << "Elapsed: " << ((clock() - startTime)/(double)CLOCKS_PER_SEC) << std::endl;

    switch(visualize_cloud){
        case 0:
            break;
        case 1:
            cloud_visualize(cluster_extraction.return_original_cloud());
            break;
        case 2:
            cloud_visualize(cluster_extraction.return_vg_cloud());
            break;
        case 3:
            break;
        default:
            break;
    }
    
    return 0; 
}