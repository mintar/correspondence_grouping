#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include <pcl/common/utils.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/home/martin/code/correspondence_grouping/cloud_target_xyz_18.pcd", *cloud_in) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }

    pcl::ConcaveHull<pcl::PointXYZ> hull_calculator;
    hull_calculator.setInputCloud(cloud_in);
    hull_calculator.setAlpha(1e3f);

    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);

    hull_calculator.reconstruct(*surface_hull, polygons);
    int dim = hull_calculator.getDimension();


    IndicesPtr indices_hull(new std::vector<int>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    bb_filter.setInputCloud(cloud_in);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(surface_hull);
    bb_filter.setDim(dim);
    bb_filter.filter(*indices_hull);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inside_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> eif;
    eif.setInputCloud(cloud_in);
    eif.setIndices(indices_hull);
    eif.filter(*inside_hull);

    static int count;
    if (surface_hull->size() > 0)
        pcl::io::savePCDFileBinary("/tmp/surface_hull.pcd", *surface_hull);
    if (inside_hull->size() > 0)
        pcl::io::savePCDFileBinary("/tmp/inside_hull.pcd", *inside_hull);
    count++;

    std::cerr << "cluster points: " << cloud_in->size() << " inside hull: " << indices_hull->size() << std::endl;

    // draw the cloud and the box
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud_in, "cluster_cloud");
    viewer.addPointCloud(surface_hull, "surface_hull");
    viewer.addPointCloud(inside_hull, "inside_hull");
    viewer.addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "convex_hull");
    viewer.spin();
}

