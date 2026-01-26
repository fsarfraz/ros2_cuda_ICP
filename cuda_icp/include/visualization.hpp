#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <Eigen/Dense>
#include <string>

/**
 * Visualize ICP alignment result
 * @param source: Original source point cloud (Nx3)
 * @param target: Target point cloud (Mx3)
 * @param aligned: Aligned source point cloud (Nx3)
 * @param transformation: Final transformation matrix (4x4)
 * @param window_name: Title for the visualization window
 */
void visualizeAlignment(const Eigen::MatrixXf& source,
                        const Eigen::MatrixXf& target,
                        const Eigen::MatrixXf& aligned,
                        const Eigen::Matrix4f& transformation,
                        const std::string& window_name = "ICP Alignment");

#endif // VISUALIZATION_HPP
