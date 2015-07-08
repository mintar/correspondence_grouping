#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(25.587, 7.8600001, 0.29100001));
  cloud->push_back(pcl::PointXYZ(24.372999, 8.5909996, 0.28799999));

  pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  boundingbox_ptr->push_back(pcl::PointXYZ(22.9035, 7.80552, -1.79216));
  boundingbox_ptr->push_back(pcl::PointXYZ(27.4452, 7.61189, -1.79216));
  boundingbox_ptr->push_back(pcl::PointXYZ(27.5215, 9.40126, -1.79216));
  boundingbox_ptr->push_back(pcl::PointXYZ(22.9798, 9.59489, -1.79216));
  boundingbox_ptr->push_back(pcl::PointXYZ(22.9035, 7.80552, 0.483439));
  boundingbox_ptr->push_back(pcl::PointXYZ(27.4452, 7.61189, 0.483439));
  boundingbox_ptr->push_back(pcl::PointXYZ(27.5215, 9.40126, 0.483439));
  boundingbox_ptr->push_back(pcl::PointXYZ(22.9798, 9.59489, 0.483439));

  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(boundingbox_ptr);
  hull.setDimension(3);
  std::vector<pcl::Vertices> polygons;

  pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZ>);
  hull.reconstruct(*surface_hull, polygons);

  pcl::PointCloud<pcl::PointXYZ>::Ptr objects (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropHull<pcl::PointXYZ> bb_filter;

  bb_filter.setInputCloud(cloud);
  bb_filter.setHullIndices(polygons);
  bb_filter.setHullCloud(boundingbox_ptr);
  bb_filter.filter(*objects);
  std::cout << objects->size() << std::endl;
}

