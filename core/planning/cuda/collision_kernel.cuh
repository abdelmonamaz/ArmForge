#pragma once

// Plain C++ declarations only — this header is included from both .cu
// (compiled by nvcc) and .cpp (compiled by MSVC) translation units, so it
// must stay free of any CUDA-specific syntax (__global__, <<<>>>, etc.).
namespace af::cuda {

// True if a usable CUDA device is present at runtime.
bool isAvailable();

// Validates `numConfigs` configurations in a single GPU launch. Each config
// is represented by `pointsPerConfig` 3-D sample points, flattened
// config-major and row-major in `points`:
//   [c0p0.x, c0p0.y, c0p0.z, c0p1.x, ..., c1p0.x, ...]
//
// validOut[i] is set to 1 if every sample point of config i clears every
// sphere obstacle (`obstacleCenters`/`obstacleRadii`, `numObstacles` spheres)
// by at least `margin`, 0 otherwise.
//
// All pointers are host memory; device buffers are allocated and freed
// internally. Throws std::runtime_error if a CUDA call fails.
void batchValidate(const float* points, int numConfigs, int pointsPerConfig,
                   const float* obstacleCenters, const float* obstacleRadii, int numObstacles,
                   float margin, unsigned char* validOut);

} // namespace af::cuda
