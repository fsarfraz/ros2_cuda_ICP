#include "icp_cpu.hpp"
#include "icp_cuda.cuh"
#include <Eigen/SVD>
#include <iostream>
#include <cmath>
#include <cfloat>

ICPResult computeICP(const Eigen::MatrixXf& source,
                     const Eigen::MatrixXf& target,
                     const ICPConfig& config) {
    ICPResult result;
    result.transformation = Eigen::Matrix4f::Identity();
    result.converged = false;

    const int n_src = source.rows();
    const int n_tgt = target.rows();

    if (config.verbose) {
        std::cout << "ICP: " << n_src << " source points, " << n_tgt << " target points\n";
        std::cout << "Max iterations: " << config.max_iterations << "\n";
        std::cout << "Convergence threshold: " << config.convergence_threshold << "\n\n";
    }

    // Allocate GPU memory
    float3* d_src_original;
    float3* d_src_transformed;
    float3* d_src_centered;
    float3* d_tgt;
    float3* d_tgt_centered;
    int* d_correspondences;
    float* d_distances;
    float* d_transformation;
    float3* d_centroid;
    float* d_H_matrix;
    float* d_error;

    CUDA_CHECK(cudaMalloc(&d_src_original, n_src * sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_src_transformed, n_src * sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_src_centered, n_src * sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_tgt, n_tgt * sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_tgt_centered, n_tgt * sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_correspondences, n_src * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_distances, n_src * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_transformation, 16 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_centroid, sizeof(float3)));
    CUDA_CHECK(cudaMalloc(&d_H_matrix, 9 * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_error, sizeof(float)));

    // Convert Eigen matrices to float3 arrays and transfer to GPU
    std::vector<float3> h_src(n_src), h_tgt(n_tgt);
    for (int i = 0; i < n_src; i++) {
        h_src[i] = make_float3(source(i, 0), source(i, 1), source(i, 2));
    }
    for (int i = 0; i < n_tgt; i++) {
        h_tgt[i] = make_float3(target(i, 0), target(i, 1), target(i, 2));
    }

    CUDA_CHECK(cudaMemcpy(d_src_original, h_src.data(), n_src * sizeof(float3),
                          cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_tgt, h_tgt.data(), n_tgt * sizeof(float3),
                          cudaMemcpyHostToDevice));

    // Initialize transformation as identity
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float h_T[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            h_T[i * 4 + j] = T(i, j);
        }
    }
    CUDA_CHECK(cudaMemcpy(d_transformation, h_T, 16 * sizeof(float),
                          cudaMemcpyHostToDevice));

    float prev_error = FLT_MAX;

    // ICP iteration loop
    for (int iter = 0; iter < config.max_iterations; iter++) {
        // Step 1: Transform source points
        launchTransformPoints(d_src_original, d_src_transformed, n_src, d_transformation);

        // Step 2: Find nearest neighbors
        launchFindNearestNeighbors(d_src_transformed, n_src, d_tgt, n_tgt,
                                    d_correspondences, d_distances);

        // Step 3: Compute centroids
        float3 centroid_src, centroid_tgt;
        launchComputeCentroid(d_src_transformed, n_src, d_centroid);
        CUDA_CHECK(cudaMemcpy(&centroid_src, d_centroid, sizeof(float3),
                              cudaMemcpyDeviceToHost));

        // Compute centroid of corresponding target points
        // For simplicity, we'll extract corresponding target points on CPU
        std::vector<int> h_correspondences(n_src);
        CUDA_CHECK(cudaMemcpy(h_correspondences.data(), d_correspondences, n_src * sizeof(int),
                              cudaMemcpyDeviceToHost));

        centroid_tgt = make_float3(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < n_src; i++) {
            centroid_tgt.x += h_tgt[h_correspondences[i]].x;
            centroid_tgt.y += h_tgt[h_correspondences[i]].y;
            centroid_tgt.z += h_tgt[h_correspondences[i]].z;
        }
        centroid_tgt.x /= n_src;
        centroid_tgt.y /= n_src;
        centroid_tgt.z /= n_src;

        // Step 4: Center point sets
        // Copy transformed source to centered buffer and center
        CUDA_CHECK(cudaMemcpy(d_src_centered, d_src_transformed, n_src * sizeof(float3),
                              cudaMemcpyDeviceToDevice));
        launchCenterPoints(d_src_centered, n_src, centroid_src);

        // Center target points (for corresponding points, we use original target centered)
        CUDA_CHECK(cudaMemcpy(d_tgt_centered, d_tgt, n_tgt * sizeof(float3),
                              cudaMemcpyDeviceToDevice));
        launchCenterPoints(d_tgt_centered, n_tgt, centroid_tgt);

        // Step 5: Compute cross-covariance matrix H
        launchComputeCrossCovariance(d_src_centered, d_tgt_centered, d_correspondences,
                                      n_src, d_H_matrix);

        // Copy H matrix to CPU
        float h_H[9];
        CUDA_CHECK(cudaMemcpy(h_H, d_H_matrix, 9 * sizeof(float), cudaMemcpyDeviceToHost));

        // Convert to Eigen matrix (row-major to column-major)
        Eigen::Matrix3f H;
        H << h_H[0], h_H[1], h_H[2],
             h_H[3], h_H[4], h_H[5],
             h_H[6], h_H[7], h_H[8];

        // Step 6: Solve for rotation using SVD
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f U = svd.matrixU();
        Eigen::Matrix3f V = svd.matrixV();
        Eigen::Matrix3f R = V * U.transpose();

        // Ensure proper rotation (det(R) = 1)
        if (R.determinant() < 0) {
            V.col(2) *= -1;
            R = V * U.transpose();
        }

        // Step 7: Compute translation
        Eigen::Vector3f centroid_src_vec(centroid_src.x, centroid_src.y, centroid_src.z);
        Eigen::Vector3f centroid_tgt_vec(centroid_tgt.x, centroid_tgt.y, centroid_tgt.z);
        Eigen::Vector3f t = centroid_tgt_vec - R * centroid_src_vec;

        // Step 8: Update transformation
        Eigen::Matrix4f T_iter = Eigen::Matrix4f::Identity();
        T_iter.block<3,3>(0,0) = R;
        T_iter.block<3,1>(0,3) = t;

        T = T_iter * T;

        // Update transformation on GPU
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                h_T[i * 4 + j] = T(i, j);
            }
        }
        CUDA_CHECK(cudaMemcpy(d_transformation, h_T, 16 * sizeof(float),
                              cudaMemcpyHostToDevice));

        // Step 9: Compute alignment error
        launchComputeError(d_src_transformed, d_tgt, d_correspondences, n_src, d_error);

        float current_error;
        CUDA_CHECK(cudaMemcpy(&current_error, d_error, sizeof(float), cudaMemcpyDeviceToHost));
        current_error = std::sqrt(current_error / n_src);  // RMS error

        result.error_history.push_back(current_error);

        if (config.verbose && (iter % 5 == 0 || iter == config.max_iterations - 1)) {
            std::cout << "Iteration " << iter << ": RMSE = " << current_error << "\n";
        }

        // Step 10: Check convergence
        float error_change = std::abs(prev_error - current_error);
        if (error_change < config.convergence_threshold) {
            result.converged = true;
            result.iterations = iter + 1;
            if (config.verbose) {
                std::cout << "\nConverged after " << result.iterations << " iterations\n";
            }
            break;
        }

        prev_error = current_error;
        result.iterations = iter + 1;
    }

    if (!result.converged && config.verbose) {
        std::cout << "\nReached maximum iterations without convergence\n";
    }

    result.transformation = T;
    result.final_error = prev_error;

    // Cleanup GPU memory
    CUDA_CHECK(cudaFree(d_src_original));
    CUDA_CHECK(cudaFree(d_src_transformed));
    CUDA_CHECK(cudaFree(d_src_centered));
    CUDA_CHECK(cudaFree(d_tgt));
    CUDA_CHECK(cudaFree(d_tgt_centered));
    CUDA_CHECK(cudaFree(d_correspondences));
    CUDA_CHECK(cudaFree(d_distances));
    CUDA_CHECK(cudaFree(d_transformation));
    CUDA_CHECK(cudaFree(d_centroid));
    CUDA_CHECK(cudaFree(d_H_matrix));
    CUDA_CHECK(cudaFree(d_error));

    return result;
}

Eigen::MatrixXf applyTransformation(const Eigen::MatrixXf& points,
                                    const Eigen::Matrix4f& transformation) {
    Eigen::MatrixXf result(points.rows(), 3);

    for (int i = 0; i < points.rows(); i++) {
        Eigen::Vector4f p_homogeneous;
        p_homogeneous << points(i, 0), points(i, 1), points(i, 2), 1.0f;

        Eigen::Vector4f p_transformed = transformation * p_homogeneous;

        result(i, 0) = p_transformed(0);
        result(i, 1) = p_transformed(1);
        result(i, 2) = p_transformed(2);
    }

    return result;
}
