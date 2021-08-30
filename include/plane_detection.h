#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/extract_indices.h>
#include <string>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;

class PlaneDetection
{
private:
    double passthrough_x_min;
    double passthrough_x_max;
    double passthrough_y_min;
    double passthrough_y_max;
    double cluster_tolerance;
    double voxel_leaf_size;
    int min_cluster_size;
    int max_cluster_size;

    pcl::PassThrough<PointT> passthrough_filter;
    pcl::VoxelGrid<PointT> voxel_filter;
    //pcl::SACSegmentationFromNormals<PointT, PointNT> sac_segmentation;
    pcl::SACSegmentation<PointT> sac_segmentation;
    pcl::ProjectInliers<PointT> project_inliers;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    pcl::NormalEstimation<PointT, PointNT> normal_estimation;
    pcl::ConvexHull<PointT> convex_hull;
    pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;

    PointCloud::Ptr getPlane(const PointCloud::Ptr &full_cloud, const Eigen::Vector3f &axis, pcl::ModelCoefficients::Ptr &coefficients, 
                             pcl::PlanarPolygon<PointT>::Ptr &hull_polygon, PointCloud::Ptr &hull_pointcloud);


public:
    PlaneDetection();
    virtual ~PlaneDetection();
    void init(double passthrough_x_min, double passthrough_x_max,
              double passthrough_y_min, double passthrough_y_max,
              double voxel_leaf_size,
              double cluster_tolerance, int min_cluster_size, int max_cluster_size);
    bool detectAndViewPlane(const std::string &filename);
};

#endif /* PLANE_DETECTION_H */

