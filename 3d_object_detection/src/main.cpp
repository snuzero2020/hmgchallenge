#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <vector>
#include "cluster_class.h"
#include "cloud_visualize.h"

int main(int argc, char** argv){
    int visualize_cloud;
    bool use_voxelgrid;
    std::string temp_data_path;

    ros::init(argc, argv, "object_detection_main");
    ros::param::get("/visualize_cloud", visualize_cloud);
    ros::param::get("/use_voxelgrid", use_voxelgrid);
    ros::param::get("/temp_data_path", temp_data_path);

    clock_t startTime = clock();
    
    ClusterExtraction cluster_extraction(use_voxelgrid, temp_data_path);

    if(use_voxelgrid)
        cluster_extraction.voxelgrid_filter();
    cluster_extraction.ransac_plane_remove();
    cluster_extraction.euclidean_cluster();

    std::cout << "Elapsed: " << ((clock() - startTime)/(double)CLOCKS_PER_SEC) << std::endl << std::endl;

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
            cloud_visualize(cluster_extraction.return_plane_removed());
            break;
        case 4:
            cluster_cloud_visualize(cluster_extraction.return_plane_removed(), cluster_extraction.return_cluster_indices());
            break;
        default:
            break;
    }
    
    return 0; 
}