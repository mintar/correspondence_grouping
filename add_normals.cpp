#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

// adds normals to pcd file
int main(int argc, char **argv)
{
    // Point clouds
    PointCloudT::Ptr cloud_in(new PointCloudT);
    PointCloudNT::Ptr normals(new PointCloudNT);

    // Load input file
    if (argc != 3)
    {
        pcl::console::print_error ("Syntax is: %s input.pcd output.pcd\n", argv[0]);
        return -1;
    }

    if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_in) < 0)
    {
        PCL_ERROR("Error loading input file!\n");
        return -1;
    }

    // compute normals
    pcl::copyPointCloud(*cloud_in, *normals);    // copy xyz to normals
    pcl::NormalEstimationOMP<PointT,PointNT> nest;
    nest.setRadiusSearch(0.025);
    nest.setInputCloud(cloud_in);
    nest.compute(*normals);

    // visualize point cloud + normals
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_in, 0, 255, 0);
    pcl::visualization::PCLVisualizer vis("cloud + normals");
    vis.addPointCloud<PointT>(cloud_in, green, "cloud_in");
    vis.addPointCloudNormals<PointT, PointNT>(cloud_in, normals, 1, 0.02f, "normals");
    vis.spin();

    // Write PCD file
    pcl::io::savePCDFileASCII(argv[2], *normals);

    return 0;
}

