#ifndef ICP_CUDA_CUH
#define ICP_CUDA_CUH

#include <cuda_runtime.h>

// Helper function to check CUDA errors
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, \
                    cudaGetErrorString(error)); \
            exit(EXIT_FAILURE); \
        } \
    } while(0)

// CUDA kernel declarations

/**
 * Transform points using a 4x4 transformation matrix
 * @param points_in: Input points (device memory)
 * @param points_out: Output transformed points (device memory)
 * @param n: Number of points
 * @param T: 4x4 transformation matrix in row-major order (device memory)
 */
void launchTransformPoints(const float3* points_in, float3* points_out, int n, const float* T);

/**
 * Find nearest neighbors using brute-force distance computation
 * @param src: Source points (device memory)
 * @param n_src: Number of source points
 * @param tgt: Target points (device memory)
 * @param n_tgt: Number of target points
 * @param indices: Output nearest neighbor indices (device memory)
 * @param distances: Output distances to nearest neighbors (device memory)
 */
void launchFindNearestNeighbors(const float3* src, int n_src,
                                 const float3* tgt, int n_tgt,
                                 int* indices, float* distances);

/**
 * Compute centroid of a point cloud using parallel reduction
 * @param points: Input points (device memory)
 * @param n: Number of points
 * @param centroid: Output centroid (device memory, 3 floats)
 */
void launchComputeCentroid(const float3* points, int n, float3* centroid);

/**
 * Center points by subtracting the centroid
 * @param points: Points to center (in-place modification, device memory)
 * @param n: Number of points
 * @param centroid: Centroid to subtract (host memory, 3 floats)
 */
void launchCenterPoints(float3* points, int n, const float3& centroid);

/**
 * Compute cross-covariance matrix H for point-to-point ICP
 * @param src_centered: Centered source points (device memory)
 * @param tgt_centered: Centered target points (device memory)
 * @param correspondences: Indices of corresponding target points (device memory)
 * @param n: Number of correspondences
 * @param H_matrix: Output 3x3 matrix in row-major order (device memory, 9 floats)
 */
void launchComputeCrossCovariance(const float3* src_centered,
                                   const float3* tgt_centered,
                                   const int* correspondences,
                                   int n,
                                   float* H_matrix);

/**
 * Compute alignment error (sum of squared distances)
 * @param src: Source points (device memory)
 * @param tgt: Target points (device memory)
 * @param correspondences: Indices of corresponding target points (device memory)
 * @param n: Number of correspondences
 * @param d_error: Output error value (device memory, 1 float)
 */
void launchComputeError(const float3* src,
                        const float3* tgt,
                        const int* correspondences,
                        int n,
                        float* d_error);

#endif // ICP_CUDA_CUH
