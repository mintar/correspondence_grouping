#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include <pcl/common/utils.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
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

    // --- filter out NaNs
    IndicesPtr indices_valid (new std::vector<int>);

    pcl::PassThrough<pcl::PointXYZ> pass_rgba;
    pass_rgba.setInputCloud(cloud_in);
    pass_rgba.setFilterFieldName("x");
    pass_rgba.setFilterLimits(-std::numeric_limits<float>::max(), std::numeric_limits<float>::max());   // accepts any real value (not inf/NaN)
    pass_rgba.setFilterLimitsNegative(false);
    pass_rgba.filter(*indices_valid);

    IndicesPtr indices (new std::vector<int>);
    for (size_t k = 0; k < cloud_in->size(); ++k)
        indices->push_back(k);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                            inside_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> eif;
    eif.setInputCloud(cloud_in);
    eif.setIndices(indices);
    eif.filter(*cluster_cloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cluster_cloud);
    //hull.setIndices(indices);
    hull.setDimension(3);
    std::vector<pcl::Vertices> polygons;

    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);

    hull.reconstruct(*surface_hull, polygons);

    IndicesPtr indices_hull(new std::vector<int>);
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    bb_filter.setInputCloud(cloud_in);
    bb_filter.setIndices(indices_valid);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(cluster_cloud);
    bb_filter.filter(*indices_hull);

    eif.setIndices(indices_hull);
    eif.filter(*inside_hull);

    static int count;
    if (surface_hull->size() > 0)
        pcl::io::savePCDFileBinary("/tmp/surface_hull.pcd", *surface_hull);
    if (cluster_cloud->size() > 0)
        pcl::io::savePCDFileBinary("/tmp/cluster_cloud.pcd", *cluster_cloud);
    if (inside_hull->size() > 0)
        pcl::io::savePCDFileBinary("/tmp/inside_hull.pcd", *inside_hull);
    count++;

    std::cerr << "cluster points: " << indices->size() << " inside hull: " << indices_hull->size() << std::endl;

    // draw the cloud and the box
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cluster_cloud, "cluster_cloud");
    viewer.addPointCloud(surface_hull, "surface_hull");
    viewer.addPointCloud(inside_hull, "inside_hull");
    viewer.addPolygonMesh<pcl::PointXYZ>(cluster_cloud, polygons, "convex_hull");
    viewer.spin();
}

