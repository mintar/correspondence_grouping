#include <iostream>
#include <string>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

// adds normals to pcd file
int main(int argc, char **argv)
{
    // Point clouds
    PointCloudNT::Ptr cloud_in(new PointCloudNT);
    PointCloudNT::Ptr cloud_in2(new PointCloudNT);

    // Load PCD files
    if (argc != 3)
    {
        pcl::console::print_error ("Syntax is: %s input1.pcd input2.pcd\n", argv[0]);
        return -1;
    }

    if (pcl::io::loadPCDFile<PointNT> (argv[1], *cloud_in) < 0)
    {
        PCL_ERROR("Error loading input file!\n");
        return -1;
    }

    if (pcl::io::loadPCDFile<PointNT> (argv[2], *cloud_in2) < 0)
    {
        PCL_ERROR("Error loading input file!\n");
        return -1;
    }

    // visualize point cloud + normals
    pcl::visualization::PointCloudColorHandlerCustom<PointNT> green (cloud_in, 0, 255, 0);
    pcl::visualization::PCLVisualizer vis("cloud + normals");

    vis.addPointCloud<PointNT>(cloud_in, green, "cloud_in");
    vis.addPointCloudNormals<PointNT, PointNT>(cloud_in, cloud_in, 1, 0.02f, "normals");

    vis.addPointCloud<PointNT>(cloud_in2, green, "cloud_in2");
    vis.addPointCloudNormals<PointNT, PointNT>(cloud_in2, cloud_in2, 1, 0.02f, "normals2");

    vis.spin();

    return 0;
}

