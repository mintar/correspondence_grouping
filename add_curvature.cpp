#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/principal_curvatures.h>


typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

// computes and saves the curvature field for point clouds which have normals, but not curvature
int main(int argc, char* argv[])
{
    // The point clouds we will be using
    PointCloudNT::Ptr cloud_in (new PointCloudNT);  // Original point cloud

    // Checking program arguments
    if (argc < 2)
    {
      printf ("Usage :\n");
      printf ("\t\t%s input.pcd output.pcd\n", argv[0]);
      PCL_ERROR ("Provide two file names.\n");
      return (-1);
    }

    if (pcl::io::loadPCDFile(argv[1], *cloud_in) < 0)
    {
      PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
      return (-1);
    }

    // curvatures are missing from CAD model point cloud -> compute (see http://www.pcl-users.org/Populating-curvature-field-Normals-already-exist-td4031631.html)
    pcl::PrincipalCurvaturesEstimation<PointNT,PointNT,pcl::PrincipalCurvatures> pc;
    pcl::PointCloud<pcl::PrincipalCurvatures> cloud_c;

    pc.setInputCloud(cloud_in);
    pc.setInputNormals(cloud_in);
    pc.setRadiusSearch(0.025);
    pc.compute(cloud_c);

    // Add the curvature field to the original point cloud
    // TODO: this doesn't do exactly the same thing as when computing the normals; compare:
    // - features/include/pcl/features/impl/feature.hpp:84
    // - features/include/pcl/features/impl/principal_curvatures.hpp:107
    for (size_t i = 0; i < cloud_in->points.size(); i++)
        cloud_in->points[i].curvature = 0.5 * (cloud_c.points[i].pc1 + cloud_c.points[i].pc2);

    pcl::io::savePCDFileBinary(argv[2], *cloud_in);
}
