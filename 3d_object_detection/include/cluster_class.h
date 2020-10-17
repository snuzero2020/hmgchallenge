#ifndef CLUSTER_CLASS
#define CLUSTER_CLASS

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
#include "ros/ros.h"

class ClusterExtraction
{
    private:
        ros::NodeHandle nh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr vg_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_removed;
        std::vector<pcl::PointIndices> cluster_indices;
        bool use_voxelgrid;
        std::string temp_data_path;

    public:
        ClusterExtraction(bool use_voxelgrid, std::string temp_data_path);
        pcl::PointCloud<pcl::PointXYZ>::Ptr return_original_cloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr return_vg_cloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr return_cloud_plane();
        pcl::PointCloud<pcl::PointXYZ>::Ptr return_plane_removed();
        std::vector<pcl::PointIndices> return_cluster_indices();
        void voxelgrid_filter();
        void ransac_plane_remove();
        void euclidean_cluster();
};


#endif