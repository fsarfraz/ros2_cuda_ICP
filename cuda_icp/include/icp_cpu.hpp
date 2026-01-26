#ifndef ICP_CPU_HPP
#define ICP_CPU_HPP

#include <Eigen/Dense>
#include <vector>

/**
 * ICP Configuration parameters
 */
struct ICPConfig {
    int max_iterations = 30;
    float convergence_threshold = 1e-6f;
    bool verbose = true;
};

/**
 * ICP Result structure
 */
struct ICPResult {
    Eigen::Matrix4f transformation;
    int iterations;
    float final_error;
    bool converged;
    std::vector<float> error_history;
};

/**
 * Point-to-point ICP algorithm using CUDA acceleration
 * @param source: Source point cloud (Nx3 matrix)
 * @param target: Target point cloud (Mx3 matrix)
 * @param config: ICP configuration parameters
 * @return ICPResult containing transformation and convergence info
 */
ICPResult computeICP(const Eigen::MatrixXf& source,
                     const Eigen::MatrixXf& target,
                     const ICPConfig& config = ICPConfig());

/**
 * Apply transformation to point cloud
 * @param points: Input point cloud (Nx3 matrix)
 * @param transformation: 4x4 transformation matrix
 * @return Transformed point cloud (Nx3 matrix)
 */
Eigen::MatrixXf applyTransformation(const Eigen::MatrixXf& points,
                                    const Eigen::Matrix4f& transformation);

#endif // ICP_CPU_HPP
