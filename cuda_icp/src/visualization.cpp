#include "visualization.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <sstream>
#include <iomanip>

void visualizeAlignment(const Eigen::MatrixXf& source,
                        const Eigen::MatrixXf& target,
                        const Eigen::MatrixXf& aligned,
                        const Eigen::Matrix4f& transformation,
                        const std::string& window_name) {
    // Convert Eigen matrices to PCL point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    source_cloud->points.resize(source.rows());
    for (int i = 0; i < source.rows(); i++) {
        source_cloud->points[i].x = source(i, 0);
        source_cloud->points[i].y = source(i, 1);
        source_cloud->points[i].z = source(i, 2);
    }

    target_cloud->points.resize(target.rows());
    for (int i = 0; i < target.rows(); i++) {
        target_cloud->points[i].x = target(i, 0);
        target_cloud->points[i].y = target(i, 1);
        target_cloud->points[i].z = target(i, 2);
    }

    aligned_cloud->points.resize(aligned.rows());
    for (int i = 0; i < aligned.rows(); i++) {
        aligned_cloud->points[i].x = aligned(i, 0);
        aligned_cloud->points[i].y = aligned(i, 1);
        aligned_cloud->points[i].z = aligned(i, 2);
    }

    // Create visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer(window_name));
    viewer->setBackgroundColor(0.05, 0.05, 0.05);

    // Add original source cloud (red)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(
        source_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_color, "source");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");

    // Add target cloud (green)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(
        target_cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");

    // Add aligned cloud (blue)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(
        aligned_cloud, 0, 100, 255);
    viewer->addPointCloud<pcl::PointXYZ>(aligned_cloud, aligned_color, "aligned");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "aligned");

    // Add coordinate system
    viewer->addCoordinateSystem(1.0);

    // Create transformation matrix text
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);
    oss << "Transformation Matrix:\n";
    for (int i = 0; i < 4; i++) {
        oss << "[";
        for (int j = 0; j < 4; j++) {
            oss << std::setw(8) << transformation(i, j);
            if (j < 3) oss << ", ";
        }
        oss << "]\n";
    }

    // Add text annotations
    viewer->addText("Red: Original Source", 10, 100, 20, 1.0, 0.0, 0.0, "source_text");
    viewer->addText("Green: Target", 10, 70, 20, 0.0, 1.0, 0.0, "target_text");
    viewer->addText("Blue: Aligned Source", 10, 40, 20, 0.0, 0.4, 1.0, "aligned_text");
    viewer->addText(oss.str(), 10, 200, 10, 1.0, 1.0, 1.0, "transform_text");

    // Set camera position for better view
    viewer->initCameraParameters();

    std::cout << "\n=== Visualization Controls ===\n";
    std::cout << "- Left mouse: Rotate\n";
    std::cout << "- Right mouse: Zoom\n";
    std::cout << "- Middle mouse: Pan\n";
    std::cout << "- 'q': Quit\n";
    std::cout << "- 'r': Reset camera\n";
    std::cout << "==============================\n\n";

    // Run visualization loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}
