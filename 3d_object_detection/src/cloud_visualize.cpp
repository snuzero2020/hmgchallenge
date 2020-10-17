#include "cloud_visualize.h"
#include <string>

using namespace std;

int color[9][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 255, 0}, {255, 0, 255}, {0, 255, 255}, {255, 0, 120}, {0, 255, 120}, {120, 255, 0}};

void cloud_visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::visualization::CloudViewer viewer("Point Cloud viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped ()){}
}

void cluster_cloud_visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices> cluster_indices){

    int j = 0;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud)[*pit]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cluster, color[j%9][0], color[j%9][1], color[j%9][2]);
        viewer->addPointCloud<pcl::PointXYZ> (cloud_cluster, single_color, "cluster_" + to_string(j));
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_" + to_string(j));
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        j++;
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}