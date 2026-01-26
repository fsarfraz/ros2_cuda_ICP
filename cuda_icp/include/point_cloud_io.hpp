#ifndef POINT_CLOUD_IO_HPP
#define POINT_CLOUD_IO_HPP

#include <Eigen/Dense>
#include <string>

/**
 * Read point cloud from text file (x y z per line)
 * @param filename: Path to input file
 * @return Point cloud as Nx3 matrix
 */
Eigen::MatrixXf readPointCloud(const std::string& filename);

/**
 * Write point cloud to text file (x y z per line)
 * @param filename: Path to output file
 * @param points: Point cloud as Nx3 matrix
 */
void writePointCloud(const std::string& filename, const Eigen::MatrixXf& points);

/**
 * Generate a cube point cloud for testing
 * @param n_points: Number of points to generate
 * @param size: Size of the cube
 * @return Point cloud as Nx3 matrix
 */
Eigen::MatrixXf generateCube(int n_points, float size = 1.0f);

/**
 * Generate a sphere point cloud for testing
 * @param n_points: Number of points to generate
 * @param radius: Radius of the sphere
 * @return Point cloud as Nx3 matrix
 */
Eigen::MatrixXf generateSphere(int n_points, float radius = 1.0f);

/**
 * Apply a transformation to generate test data
 * @param points: Input point cloud
 * @param translation: Translation vector (x, y, z)
 * @param rotation_deg: Rotation angles in degrees (roll, pitch, yaw)
 * @param noise_std: Standard deviation of Gaussian noise to add
 * @return Transformed point cloud
 */
Eigen::MatrixXf transformPointCloud(const Eigen::MatrixXf& points,
                                    const Eigen::Vector3f& translation,
                                    const Eigen::Vector3f& rotation_deg,
                                    float noise_std = 0.0f);

#endif // POINT_CLOUD_IO_HPP
