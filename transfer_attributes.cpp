#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

using namespace pcl::console;
using namespace pcl;

void
printHelp(int, char **argv)
{
  print_error("Syntax is: %s point_input.pcd attribute_input.pcd output.pcd\n", argv[0]);
  print_info("\n");
  print_info("Each point in point_input.pcd is augmented with the color, normal and curvature attributes\n");
  print_info("of the closest point in attribute_input.pcd, and the result will be copied to output.pcd.\n");
}

int
main(int argc, char **argv)
{
  print_info("Transfer the color, normal and curvature attributes from one point cloud_in to another.\n");
  print_info("For more information, use: %s -h\n", argv[0]);

  std::vector<int> pcd_file_indices = parse_file_extension_argument(argc, argv, ".pcd");

  if (pcd_file_indices.size() != 3)
  {
    printHelp(argc, argv);
    return (-1);
  }

  // load files
  PointCloud<PointXYZ>::Ptr cloud_in(new PointCloud<PointXYZ>);
  if (pcl::io::loadPCDFile(argv[pcd_file_indices[0]], *cloud_in) != 0)
  {
    print_error("Could not load file: %s\n", argv[pcd_file_indices[0]]);
    return (-1);
  }
  PointCloud<PointXYZRGBNormal>::Ptr cloud_attrs(new PointCloud<PointXYZRGBNormal>);
  if (pcl::io::loadPCDFile(argv[pcd_file_indices[1]], *cloud_attrs) != 0)
  {
    print_error("Could not load file: %s\n", argv[pcd_file_indices[1]]);
    return (-1);
  }

  PointCloud<PointXYZRGBNormal>::Ptr cloud_out(new PointCloud<PointXYZRGBNormal>);
  copyPointCloud(*cloud_in, *cloud_out);

  // K nearest neighbor search
  pcl::KdTreeFLANN<PointXYZRGBNormal> kdtree;
  kdtree.setInputCloud(cloud_attrs);
  int K = 1;
  std::vector<int> nearest_point_indices(K);
  std::vector<float> sqr_distances(K);

  size_t num_invalid_points = 0;
  for (size_t i = 0; i < cloud_in->size(); ++i)
  {
    const auto &pt_in = cloud_in->at(i);

    PointXYZRGBNormal search_point;
    copyPoint(pt_in, search_point);

    if (kdtree.nearestKSearch(search_point, K, nearest_point_indices, sqr_distances) == 0
        || sqr_distances[0] > 0.001)
    {
      ++num_invalid_points;
      continue;
    }

    const PointXYZRGBNormal &pt_attrs = cloud_attrs->at(nearest_point_indices[0]);
    auto &pt_out = cloud_out->at(i);
    pt_out.rgba = pt_attrs.rgba;
    pt_out.normal_x = pt_attrs.normal_x;
    pt_out.normal_y = pt_attrs.normal_y;
    pt_out.normal_z = pt_attrs.normal_z;
    pt_out.curvature = pt_attrs.curvature;
  }

  if (num_invalid_points > 0)
    print_warn("Could not transfer attributes for %zu out of %zu points!\n", num_invalid_points, cloud_in->size());

  if (pcl::io::savePCDFileASCII(argv[pcd_file_indices[2]], *cloud_out) != 0)
  {
    print_error("Could not save file: %s\n", argv[pcd_file_indices[2]]);
    return (-1);
  }

  return 0;
}
