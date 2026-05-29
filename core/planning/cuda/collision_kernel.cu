#include "collision_kernel.cuh"

#include <cuda_runtime.h>
#include <stdexcept>
#include <string>

namespace af::cuda {

namespace {

void check(cudaError_t err, const char* what)
{
    if (err != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA error in ") + what + ": " + cudaGetErrorString(err));
    }
}

// One thread per configuration: walk its sample points against every
// obstacle sphere and bail out at the first overlap. `numConfigs` such
// independent checks run concurrently across the GPU's SMs — the parallel
// counterpart to RrtConnectPlanner's sequential per-point CPU loop.
__global__ void k_batchValidate(const float* points, int pointsPerConfig,
                                const float* centers, const float* radii, int numObstacles,
                                float margin, unsigned char* validOut, int numConfigs)
{
    const int cfg = blockIdx.x * blockDim.x + threadIdx.x;
    if (cfg >= numConfigs) {
        return;
    }

    const float* pts = points + cfg * pointsPerConfig * 3;
    bool clear = true;
    for (int p = 0; p < pointsPerConfig && clear; ++p) {
        const float px = pts[p * 3 + 0];
        const float py = pts[p * 3 + 1];
        const float pz = pts[p * 3 + 2];
        for (int o = 0; o < numObstacles; ++o) {
            const float dx = px - centers[o * 3 + 0];
            const float dy = py - centers[o * 3 + 1];
            const float dz = pz - centers[o * 3 + 2];
            const float dist = sqrtf(dx * dx + dy * dy + dz * dz);
            if (dist < radii[o] + margin) {
                clear = false;
                break;
            }
        }
    }
    validOut[cfg] = clear ? 1 : 0;
}

} // namespace

bool isAvailable()
{
    int count = 0;
    return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

void batchValidate(const float* points, int numConfigs, int pointsPerConfig,
                   const float* obstacleCenters, const float* obstacleRadii, int numObstacles,
                   float margin, unsigned char* validOut)
{
    if (numConfigs <= 0) {
        return;
    }

    const std::size_t pointsBytes  = static_cast<std::size_t>(numConfigs) * pointsPerConfig * 3 * sizeof(float);
    const std::size_t centersBytes = static_cast<std::size_t>(numObstacles) * 3 * sizeof(float);
    const std::size_t radiiBytes   = static_cast<std::size_t>(numObstacles) * sizeof(float);
    const std::size_t validBytes   = static_cast<std::size_t>(numConfigs) * sizeof(unsigned char);

    float*         d_points  = nullptr;
    float*         d_centers = nullptr;
    float*         d_radii   = nullptr;
    unsigned char* d_valid   = nullptr;

    try {
        check(cudaMalloc(&d_points,  pointsBytes),  "cudaMalloc(points)");
        check(cudaMalloc(&d_centers, centersBytes), "cudaMalloc(centers)");
        check(cudaMalloc(&d_radii,   radiiBytes),   "cudaMalloc(radii)");
        check(cudaMalloc(&d_valid,   validBytes),   "cudaMalloc(valid)");

        check(cudaMemcpy(d_points,  points,          pointsBytes,  cudaMemcpyHostToDevice), "memcpy(points)");
        check(cudaMemcpy(d_centers, obstacleCenters, centersBytes, cudaMemcpyHostToDevice), "memcpy(centers)");
        check(cudaMemcpy(d_radii,   obstacleRadii,   radiiBytes,   cudaMemcpyHostToDevice), "memcpy(radii)");

        constexpr int kThreads = 256;
        const int blocks = (numConfigs + kThreads - 1) / kThreads;
        k_batchValidate<<<blocks, kThreads>>>(d_points, pointsPerConfig, d_centers, d_radii,
                                              numObstacles, margin, d_valid, numConfigs);
        check(cudaGetLastError(), "kernel launch");
        check(cudaDeviceSynchronize(), "kernel sync");

        check(cudaMemcpy(validOut, d_valid, validBytes, cudaMemcpyDeviceToHost), "memcpy(valid)");
    } catch (...) {
        cudaFree(d_points);
        cudaFree(d_centers);
        cudaFree(d_radii);
        cudaFree(d_valid);
        throw;
    }

    cudaFree(d_points);
    cudaFree(d_centers);
    cudaFree(d_radii);
    cudaFree(d_valid);
}

} // namespace af::cuda
