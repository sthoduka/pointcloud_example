#include "plane_detection.h"

#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>


PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}

void PlaneDetection::init(double passthrough_x_min, double passthrough_x_max,
                          double passthrough_y_min, double passthrough_y_max,
                          double voxel_leaf_size,
                          double cluster_tolerance, int min_cluster_size, int max_cluster_size)
{
    this->passthrough_x_min = passthrough_x_min; //0.0;
    this->passthrough_x_max = passthrough_x_max; //8.0;
    this->passthrough_y_min = passthrough_y_min; //-0.7;
    this->passthrough_y_max = passthrough_y_max; //0.7;
    this->voxel_leaf_size = voxel_leaf_size;
    this->cluster_tolerance = cluster_tolerance; //0.02;
    this->min_cluster_size = min_cluster_size; //500;
    this->max_cluster_size = max_cluster_size; //10000;

    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());

    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setRadiusSearch(0.10);
}

PointCloud::Ptr PlaneDetection::getPlane(const PointCloud::Ptr &full_cloud, const Eigen::Vector3f &axis,
                    pcl::ModelCoefficients::Ptr &coefficients, pcl::PlanarPolygon<PointT>::Ptr &hull_polygon,
                    PointCloud::Ptr &hull_pointcloud)
{
    PointCloud::Ptr filtered_cloud(new PointCloud);
    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(passthrough_x_min, passthrough_x_max);
    passthrough_filter.filter(*filtered_cloud);

    passthrough_filter.setInputCloud(filtered_cloud);
    passthrough_filter.setFilterFieldName("y");
    passthrough_filter.setFilterLimits(passthrough_y_min, passthrough_y_max);
    passthrough_filter.filter(*filtered_cloud);

    std::cout << "Passthrough complete " << std::endl;

    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*filtered_cloud);

    std::cout << "voxel filter complete " << std::endl;

    // indices of points which are part of the detected plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    PointCloudN::Ptr normals(new PointCloudN);
    normal_estimation.setInputCloud(filtered_cloud);
    normal_estimation.compute(*normals);
    std::cout << "Normal estimation complete " << std::endl;

    //sac_segmentation.setAxis(axis);
    //sac_segmentation.setInputNormals(normals);
    sac_segmentation.setOptimizeCoefficients(true);
    sac_segmentation.setModelType(pcl::SACMODEL_PLANE);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation.setDistanceThreshold(0.05);
    sac_segmentation.setEpsAngle(0.3);

    sac_segmentation.setInputCloud(filtered_cloud);
    sac_segmentation.segment(*inliers, *coefficients);

    std::cout << "SAC segmentation complete " << std::endl;

    if (inliers->indices.size() == 0)
    {
        std::cout << "could not find plane " << std::endl;
        return filtered_cloud;
    }

    PointCloud::Ptr plane_cloud(new PointCloud);

    project_inliers.setModelType(pcl::SACMODEL_PLANE);
    project_inliers.setInputCloud(filtered_cloud);
    project_inliers.setIndices(inliers);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.filter(*plane_cloud);

    std::cout << "Projection to plane complete " << std::endl;

    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*hull_pointcloud);
    hull_pointcloud->points.push_back(hull_pointcloud->points.front());
    hull_pointcloud->width += 1;
    hull_polygon->setContour(*hull_pointcloud);
    hull_polygon->setCoefficients(*coefficients);
    std::cout << "Convex hull computation complete " << std::endl;

    return plane_cloud;
}


bool PlaneDetection::detectAndViewPlane(const std::string &filename)
{

    PointCloud::Ptr full_cloud(new PointCloud);
    if(pcl::io::loadPCDFile<PointT>(filename, *full_cloud) == -1)
    {
        std::cout << "failed to load" << std::endl;
        return false;
    }

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PlanarPolygon<PointT>::Ptr hull_polygon(new pcl::PlanarPolygon<PointT>);
    PointCloud::Ptr hull_pointcloud(new PointCloud);

    PointCloud::Ptr plane = getPlane(full_cloud, Eigen::Vector3f(1.0, 0.0, 0.0), plane_coefficients, hull_polygon, hull_pointcloud);

    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);

    extract_polygonal_prism.setInputPlanarHull(hull_pointcloud);
    extract_polygonal_prism.setInputCloud(full_cloud);
    extract_polygonal_prism.setViewPoint(0.0, 0.0, 2.0);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    std::cout << "Extract polygon prism complete " << std::endl;

    /*
    PointCloud::Ptr prism(new PointCloud);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(full_cloud);
    extract.setIndices(segmented_cloud_inliers);
    extract.setNegative (false);
    extract.filter (*prism);
    */

    std::vector<pcl::PointIndices> clusters_indices;
    std::vector<PointCloud::Ptr> clusters;

    cluster_extraction.setInputCloud(full_cloud);
    cluster_extraction.setIndices(segmented_cloud_inliers);
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(min_cluster_size);
    cluster_extraction.setMaxClusterSize(max_cluster_size);


    cluster_extraction.extract(clusters_indices);
    std::cout << "Cluster extraction complete " << std::endl;

    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        PointCloud::Ptr cluster(new PointCloud);
        pcl::copyPointCloud(*full_cloud, cluster_indices, *cluster);
        clusters.push_back(cluster);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(plane, 0, 255, 0);
    viewer->addPointCloud<PointT> (plane, green, "plane");

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> colour_handler(full_cloud);
 //   pcl::visualization::PointCloudColorHandlerCustom<PointT> colour_handler(full_cloud, 255, 0, 0);
    viewer->addPointCloud<PointT>(full_cloud, colour_handler, "full_cloud");

    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> random_int_generator(0, 255);
    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
      pcl::visualization::PointCloudColorHandlerCustom<PointT> red(clusters[i], random_int_generator(rng), random_int_generator(rng), random_int_generator(rng));
      viewer->addPointCloud<PointT>(clusters[i], red, std::to_string(i));
    }

    viewer->addPolygon(*hull_polygon, 0.0, 1.0, 0.0, "hull", 0);
    viewer->addCoordinateSystem(0.1, "coordinate_system", 0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: ./detect_plane <path to .pcd file>" << std::endl;
        exit(0);
    }
    double passthrough_x_min;
    double passthrough_x_max;
    double passthrough_y_min;
    double passthrough_y_max;
    double voxel_leaf_size;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    passthrough_x_min = std::stod(argv[2]);
    passthrough_x_max = std::stod(argv[3]);
    passthrough_y_min = std::stod(argv[4]);
    passthrough_y_max = std::stod(argv[5]);
    voxel_leaf_size = std::stod(argv[6]);
    cluster_tolerance = std::stod(argv[7]);
    min_cluster_size = std::stoi(argv[8]);
    max_cluster_size = std::stoi(argv[9]);
    // std::stod, std::stoi
    PlaneDetection pd;
    pd.init(passthrough_x_min, passthrough_x_max, passthrough_y_min, passthrough_y_max, voxel_leaf_size, cluster_tolerance, min_cluster_size, max_cluster_size);
    pd.detectAndViewPlane(std::string(argv[1]));
    return 0;
}

