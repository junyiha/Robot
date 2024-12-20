cmake_minimum_required(VERSION 3.8)
project(Robot)

# set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(BaseDIR D:/)
set(CMAKE_PREFIX_PATH "D:/soft/qt5.14/5.14.0/msvc2017_64")
set(QRC_SOURCE_FILE ${CMAKE_SOURCE_DIR}/res.qrc)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/)

find_program(NVIDIA_SMI nvidia-smi)
if(NVIDIA_SMI)
    set(BUILD_GPU_FLAG ON)
endif()

if(BUILD_GPU_FLAG)
    message(STATUS "this is gpu version")
    add_definitions(-DGPU_FLAG=true)
endif()

add_compile_options(/W0)
if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2 /GL")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /O2 /GL")
endif()

# add_definitions(-DSTATE_MACHINE_TEST=1)
add_definitions(-DROOT_PATH="${CMAKE_SOURCE_DIR}/")
add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config.yaml")
if(BUILD_GPU_FLAG)
        add_definitions(-DVISION_MODEL_PATH="${CMAKE_SOURCE_DIR}/models/model_ocrnet-20241020.engine")
else()
        add_definitions(-DVISION_MODEL_PATH="${CMAKE_SOURCE_DIR}/models/pp_liteseg_stdc1_softmax_20241021.onnx")
endif(BUILD_GPU_FLAG)

# for vscode
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455d.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)

file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Core.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Cored.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Gui.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Guid.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgets.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgetsd.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${CMAKE_PREFIX_PATH}/plugins/platforms DESTINATION ${CMAKE_BINARY_DIR}/Debug//plugins/)
file(COPY ${BaseDIR}/packages/HK_SDK/Libraries/win64/MvCameraControl.lib DESTINATION ${CMAKE_BINARY_DIR}/Debug//)
file(COPY ${BaseDIR}/packages/BestfitLayserScaner/lib/bin/BestfitLaserScanerSDK.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug//)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug//)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_cuda.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug//)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_shared.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug//)
file(COPY ${CMAKE_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR}/Debug//)

# for clion
file(COPY ${CMAKE_PREFIX_PATH}/plugins/platforms DESTINATION ${CMAKE_BINARY_DIR}/plugins/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Core.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Cored.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Gui.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Guid.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgets.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgetsd.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/HK_SDK/Libraries/win64/MvCameraControl.lib DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/BestfitLayserScaner/lib/bin/BestfitLaserScanerSDK.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_cuda.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_shared.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${CMAKE_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455d.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455.dll DESTINATION ${CMAKE_BINARY_DIR}/)

if(BUILD_GPU_FLAG)
    # TensorRT
    set(TENSORRT_PATH ${BaseDIR}/packages/TensorRT-8.6.1.6)
    set(TENSORRT_LIBS
            ${TENSORRT_PATH}/lib/nvinfer.lib
            ${TENSORRT_PATH}/lib/nvonnxparser.lib
            ${TENSORRT_PATH}/lib/nvparsers.lib
            ${TENSORRT_PATH}/lib/nvinfer_plugin.lib
    )
    include_directories(${TENSORRT_PATH}/include)

    #CUDA
    set(CUDA_TOOLKIT_ROOT_DIR "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v11.3")
    set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/include)
    set(CUDA_LIBS_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64)
    set(CUDA_LIBS
            ${CUDA_LIBS_DIRS}/cublas.lib
            ${CUDA_LIBS_DIRS}/cudnn.lib
            ${CUDA_LIBS_DIRS}/cuda.lib
            ${CUDA_LIBS_DIRS}/cudadevrt.lib
            ${CUDA_LIBS_DIRS}/cudart.lib
    )
    include_directories(${CUDA_INCLUDE_DIRS})
    link_directories(${CUDA_LIBS_DIRS})
endif()

# Opencv Setting
include_directories(${BaseDIR}/packages/opencv4.55/build/include)
link_directories(${BaseDIR}/packages/opencv4.55/build/x64/vc15/lib)

# Spdlog Setting
include_directories(${BaseDIR}/packages)

#HK SDK  Setting
include_directories(${BaseDIR}/packages/HK_SDK/Includes)
link_directories(${BaseDIR}/packages/HK_SDK/Libraries/win64)

# onnxruntime setting
include_directories(${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/build/native/include)
link_directories(${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native)

#Bestfit laser setting
include_directories(${BaseDIR}/packages/BestfitLayserScaner/lib/include)
link_directories(${BaseDIR}/packages/BestfitLayserScaner/lib/lib)

# eigen3
include_directories(${BaseDIR}/packages/eigen3.4.0)

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/source-charset:utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")

add_subdirectory(${CMAKE_SOURCE_DIR}/ThirdParty/yaml-cpp-0.8.0)

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        Network
        Xml
        REQUIRED)

#  add resources
qt5_add_resources(${QRC_SOURCE_FILE})

file(GLOB ALL_ITEMS "${CMAKE_SOURCE_DIR}/src/*")
foreach(item ${ALL_ITEMS})
    aux_source_directory(${item} src)
endforeach()

aux_source_directory(${CMAKE_SOURCE_DIR}/ src)

add_executable(${PROJECT_NAME} ${src} ${QRC_SOURCE_FILE})

foreach(item ${ALL_ITEMS})
    target_include_directories(${PROJECT_NAME} PUBLIC ${item})
endforeach()

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/
                                                  ${CMAKE_SOURCE_DIR}/src
                                                  ${CMAKE_SOURCE_DIR}/ThirdParty/yaml-cpp-0.8.0/include/
                                                  ${CMAKE_SOURCE_DIR}/ThirdParty/cxxopts-3.2.0/include/
                                                  ${CMAKE_SOURCE_DIR}/ThirdParty/asio-1.30.2/include
)

# yaml-0.8.0
target_link_libraries(${PROJECT_NAME}   yaml-cpp::yaml-cpp
                                        Qt5::Core
                                        Qt5::Gui
                                        Qt5::Widgets
                                        Qt5::Network
                                        Qt5::Xml
                                        opencv_world455d.lib
                                        opencv_world455.dll
                                        MvCameraControl.lib
                                        onnxruntime.dll
                                        BestfitLaserScanerSDK.lib
)

# tensorrt cuda
if(BUILD_GPU_FLAG)
target_link_libraries(${PROJECT_NAME}
        ${TENSORRT_LIBS}
        ${CUDA_LIBS}
)
endif()