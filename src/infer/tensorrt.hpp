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

namespace Infer
{

    void Init();



}  // namespace Infer

#endif // GPU_FLAG