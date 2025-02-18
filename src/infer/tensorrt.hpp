/**
 *
*/
#pragma once

#include "common.hpp"

#ifdef GPU_FLAG

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include "vision/logging.h"
#include "vision/LineSegmenationBaseTensorRT.h"

namespace Infer
{

    void Init();

    void TestLineSegmenationBaseTensorRT();




}  // namespace Infer

#endif // GPU_FLAG