#include "icp_cuda.cuh"
#include <cuda_runtime.h>
#include <cfloat>
#include <cstdio>

#define BLOCK_SIZE 256

// ============================================================================
// Kernel 1: Transform Points
// ============================================================================

__global__ void transformPointsKernel(const float3* points_in, float3* points_out,
                                       int n, const float* T) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;

    float3 p = points_in[idx];

    // Apply 4x4 transformation matrix (homogeneous coordinates)
    // T is in row-major order
    float x = T[0]*p.x + T[1]*p.y + T[2]*p.z + T[3];
    float y = T[4]*p.x + T[5]*p.y + T[6]*p.z + T[7];
    float z = T[8]*p.x + T[9]*p.y + T[10]*p.z + T[11];

    points_out[idx] = make_float3(x, y, z);
}

void launchTransformPoints(const float3* points_in, float3* points_out, int n, const float* T) {
    int num_blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    transformPointsKernel<<<num_blocks, BLOCK_SIZE>>>(points_in, points_out, n, T);
    CUDA_CHECK(cudaGetLastError());
}

// ============================================================================
// Kernel 2: Find Nearest Neighbors (Brute-Force)
// ============================================================================

__global__ void findNearestNeighborsKernel(const float3* src, int n_src,
                                            const float3* tgt, int n_tgt,
                                            int* indices, float* distances) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n_src) return;

    float3 src_pt = src[idx];
    float min_dist_sq = FLT_MAX;
    int min_idx = -1;

    // Brute-force search over all target points
    for (int j = 0; j < n_tgt; j++) {
        float3 tgt_pt = tgt[j];
        float dx = src_pt.x - tgt_pt.x;
        float dy = src_pt.y - tgt_pt.y;
        float dz = src_pt.z - tgt_pt.z;
        float dist_sq = dx*dx + dy*dy + dz*dz;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            min_idx = j;
        }
    }

    indices[idx] = min_idx;
    distances[idx] = sqrtf(min_dist_sq);
}

void launchFindNearestNeighbors(const float3* src, int n_src,
                                 const float3* tgt, int n_tgt,
                                 int* indices, float* distances) {
    int num_blocks = (n_src + BLOCK_SIZE - 1) / BLOCK_SIZE;
    findNearestNeighborsKernel<<<num_blocks, BLOCK_SIZE>>>(src, n_src, tgt, n_tgt,
                                                             indices, distances);
    CUDA_CHECK(cudaGetLastError());
}

// ============================================================================
// Kernel 3: Compute Centroid (Parallel Reduction)
// ============================================================================

__global__ void computeCentroidKernel(const float3* points, int n, float3* partial_sums) {
    __shared__ float3 shared_data[BLOCK_SIZE];

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Load data from global memory
    float3 sum = make_float3(0.0f, 0.0f, 0.0f);
    if (idx < n) {
        sum = points[idx];
    }
    shared_data[tid] = sum;
    __syncthreads();

    // Reduction in shared memory
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared_data[tid].x += shared_data[tid + s].x;
            shared_data[tid].y += shared_data[tid + s].y;
            shared_data[tid].z += shared_data[tid + s].z;
        }
        __syncthreads();
    }

    // Write result for this block
    if (tid == 0) {
        partial_sums[blockIdx.x] = shared_data[0];
    }
}

void launchComputeCentroid(const float3* points, int n, float3* centroid) {
    int num_blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // Allocate temporary storage for partial sums
    float3* d_partial_sums;
    CUDA_CHECK(cudaMalloc(&d_partial_sums, num_blocks * sizeof(float3)));

    // First reduction: per-block sums
    computeCentroidKernel<<<num_blocks, BLOCK_SIZE>>>(points, n, d_partial_sums);
    CUDA_CHECK(cudaGetLastError());

    // Second reduction: sum of per-block sums (small, do on CPU)
    float3* h_partial_sums = new float3[num_blocks];
    CUDA_CHECK(cudaMemcpy(h_partial_sums, d_partial_sums, num_blocks * sizeof(float3),
                          cudaMemcpyDeviceToHost));

    float3 result = make_float3(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < num_blocks; i++) {
        result.x += h_partial_sums[i].x;
        result.y += h_partial_sums[i].y;
        result.z += h_partial_sums[i].z;
    }

    // Divide by n to get centroid
    result.x /= n;
    result.y /= n;
    result.z /= n;

    // Copy result to device
    CUDA_CHECK(cudaMemcpy(centroid, &result, sizeof(float3), cudaMemcpyHostToDevice));

    // Cleanup
    delete[] h_partial_sums;
    CUDA_CHECK(cudaFree(d_partial_sums));
}

// ============================================================================
// Kernel 4: Center Points
// ============================================================================

__global__ void centerPointsKernel(float3* points, int n, float3 centroid) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;

    points[idx].x -= centroid.x;
    points[idx].y -= centroid.y;
    points[idx].z -= centroid.z;
}

void launchCenterPoints(float3* points, int n, const float3& centroid) {
    int num_blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    centerPointsKernel<<<num_blocks, BLOCK_SIZE>>>(points, n, centroid);
    CUDA_CHECK(cudaGetLastError());
}

// ============================================================================
// Kernel 5: Compute Cross-Covariance Matrix H (Parallel Reduction)
// ============================================================================

__global__ void computeCrossCovarianceKernel(const float3* src_centered,
                                              const float3* tgt_centered,
                                              const int* correspondences,
                                              int n,
                                              float* partial_H) {
    __shared__ float shared_H[BLOCK_SIZE * 9];  // 9 elements per thread

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Initialize shared memory
    for (int i = 0; i < 9; i++) {
        shared_H[tid * 9 + i] = 0.0f;
    }

    // Compute outer product for this thread's point pair
    if (idx < n) {
        float3 s = src_centered[idx];
        float3 t = tgt_centered[correspondences[idx]];

        // H = sum(s * t^T) = sum of outer products
        shared_H[tid * 9 + 0] = s.x * t.x;
        shared_H[tid * 9 + 1] = s.x * t.y;
        shared_H[tid * 9 + 2] = s.x * t.z;
        shared_H[tid * 9 + 3] = s.y * t.x;
        shared_H[tid * 9 + 4] = s.y * t.y;
        shared_H[tid * 9 + 5] = s.y * t.z;
        shared_H[tid * 9 + 6] = s.z * t.x;
        shared_H[tid * 9 + 7] = s.z * t.y;
        shared_H[tid * 9 + 8] = s.z * t.z;
    }
    __syncthreads();

    // Reduction in shared memory
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            for (int i = 0; i < 9; i++) {
                shared_H[tid * 9 + i] += shared_H[(tid + s) * 9 + i];
            }
        }
        __syncthreads();
    }

    // Write result for this block
    if (tid == 0) {
        for (int i = 0; i < 9; i++) {
            partial_H[blockIdx.x * 9 + i] = shared_H[i];
        }
    }
}

void launchComputeCrossCovariance(const float3* src_centered,
                                   const float3* tgt_centered,
                                   const int* correspondences,
                                   int n,
                                   float* H_matrix) {
    int num_blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // Allocate temporary storage for partial H matrices
    float* d_partial_H;
    CUDA_CHECK(cudaMalloc(&d_partial_H, num_blocks * 9 * sizeof(float)));

    // First reduction: per-block H matrices
    computeCrossCovarianceKernel<<<num_blocks, BLOCK_SIZE>>>(
        src_centered, tgt_centered, correspondences, n, d_partial_H);
    CUDA_CHECK(cudaGetLastError());

    // Second reduction: sum of per-block H matrices (small, do on CPU)
    float* h_partial_H = new float[num_blocks * 9];
    CUDA_CHECK(cudaMemcpy(h_partial_H, d_partial_H, num_blocks * 9 * sizeof(float),
                          cudaMemcpyDeviceToHost));

    float result_H[9] = {0};
    for (int i = 0; i < num_blocks; i++) {
        for (int j = 0; j < 9; j++) {
            result_H[j] += h_partial_H[i * 9 + j];
        }
    }

    // Copy result to device
    CUDA_CHECK(cudaMemcpy(H_matrix, result_H, 9 * sizeof(float), cudaMemcpyHostToDevice));

    // Cleanup
    delete[] h_partial_H;
    CUDA_CHECK(cudaFree(d_partial_H));
}

// ============================================================================
// Kernel 6: Compute Alignment Error (Parallel Reduction)
// ============================================================================

__global__ void computeErrorKernel(const float3* src, const float3* tgt,
                                    const int* correspondences, int n,
                                    float* partial_errors) {
    __shared__ float shared_data[BLOCK_SIZE];

    int tid = threadIdx.x;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // Compute squared distance for this thread
    float error = 0.0f;
    if (idx < n) {
        float3 s = src[idx];
        float3 t = tgt[correspondences[idx]];
        float dx = s.x - t.x;
        float dy = s.y - t.y;
        float dz = s.z - t.z;
        error = dx*dx + dy*dy + dz*dz;
    }
    shared_data[tid] = error;
    __syncthreads();

    // Reduction in shared memory
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            shared_data[tid] += shared_data[tid + s];
        }
        __syncthreads();
    }

    // Write result for this block
    if (tid == 0) {
        partial_errors[blockIdx.x] = shared_data[0];
    }
}

void launchComputeError(const float3* src, const float3* tgt,
                        const int* correspondences, int n, float* d_error) {
    int num_blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;

    // Allocate temporary storage for partial errors
    float* d_partial_errors;
    CUDA_CHECK(cudaMalloc(&d_partial_errors, num_blocks * sizeof(float)));

    // First reduction: per-block errors
    computeErrorKernel<<<num_blocks, BLOCK_SIZE>>>(src, tgt, correspondences, n,
                                                    d_partial_errors);
    CUDA_CHECK(cudaGetLastError());

    // Second reduction: sum of per-block errors (small, do on CPU)
    float* h_partial_errors = new float[num_blocks];
    CUDA_CHECK(cudaMemcpy(h_partial_errors, d_partial_errors, num_blocks * sizeof(float),
                          cudaMemcpyDeviceToHost));

    float result = 0.0f;
    for (int i = 0; i < num_blocks; i++) {
        result += h_partial_errors[i];
    }

    // Copy result to device
    CUDA_CHECK(cudaMemcpy(d_error, &result, sizeof(float), cudaMemcpyHostToDevice));

    // Cleanup
    delete[] h_partial_errors;
    CUDA_CHECK(cudaFree(d_partial_errors));
}
