cmake_minimum_required(VERSION 3.8)
project(PDRobot-master)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# set(CMAKE_PREFIX_PATH "D:/soft/Qt5.14.0/5.14.0/msvc2017_64")
set(CMAKE_PREFIX_PATH "D:/Qt514/5.14.0/msvc2017_64")
set(BaseDIR D:/)
#set(CMAKE_PREFIX_PATH "D:/soft/qt5.14/5.14.0/msvc2017_64")
#set(BaseDIR E:/znzz/codes/qt/PDRootV1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")

set(QRC_SOURCE_FILE ${CMAKE_SOURCE_DIR}/res.qrc)

# add_definitions(-DSTATE_MACHINE_TEST=1)

# ffmpeg
#file(COPY ${BaseDIR}/packages/ffmpeg/bin/avcodec-59.dll DESTINATION ${CMAKE_BINARY_DIR})
#file(COPY ${BaseDIR}/packages/ffmpeg/bin/avformat-59.dll DESTINATION ${CMAKE_BINARY_DIR})
#file(COPY ${BaseDIR}/packages/ffmpeg/bin/avformat-59.dll DESTINATION ${CMAKE_BINARY_DIR})
#file(COPY ${BaseDIR}/packages/ffmpeg/bin/swresample-4.dll DESTINATION ${CMAKE_BINARY_DIR})

# opencv
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455d.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455.dll DESTINATION ${CMAKE_BINARY_DIR})

# Qt
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Core.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Cored.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Gui.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Guid.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgets.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgetsd.dll DESTINATION ${CMAKE_BINARY_DIR})
file(COPY ${CMAKE_PREFIX_PATH}/plugins/platforms DESTINATION ${CMAKE_BINARY_DIR}/plugins/)

# HK
file(COPY ${BaseDIR}/packages/HK_SDK/Libraries/win64/MvCameraControl.lib DESTINATION ${CMAKE_BINARY_DIR}/)

# GoSdk
#file(COPY ${BaseDIR}/packages/GocatorSDK/bin/win64/kApi.dll DESTINATION ${CMAKE_BINARY_DIR}/)
#file(COPY ${BaseDIR}/packages/GocatorSDK/bin/win64/GoSdk.dll DESTINATION ${CMAKE_BINARY_DIR}/)

# BestfitLaserScanner
file(COPY ${BaseDIR}/packages/BestfitLayserScaner/lib/bin/BestfitLaserScanerSDK.dll DESTINATION ${CMAKE_BINARY_DIR}/)

# onnxruntime
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_cuda.dll DESTINATION ${CMAKE_BINARY_DIR}/)
file(COPY ${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.Gpu.1.8.0/runtimes/win-x64/native/onnxruntime_providers_shared.dll DESTINATION ${CMAKE_BINARY_DIR}/)

# vision models
file(COPY ${CMAKE_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR}/)

# Opencv Setting
include_directories(${BaseDIR}/packages/opencv4.55/build/include)
link_directories(${BaseDIR}/packages/opencv4.55/build/x64/vc15/lib)

# Spdlog Setting
include_directories(${BaseDIR}/packages)

#HK SDK  Setting
include_directories(${BaseDIR}/packages/HK_SDK/Includes)
link_directories(${BaseDIR}/packages/HK_SDK/Libraries/win64)

## ffmpeg setting
#include_directories(${BaseDIR}/packages/ffmpeg/include)
#link_directories(${BaseDIR}/packages/ffmpeg/lib)
#
## Gocator setting
#include_directories(${BaseDIR}/packages/GocatorSDK/include)
#link_directories(${BaseDIR}/packages/GocatorSDK/lib/win64d)

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

aux_source_directory(${CMAKE_SOURCE_DIR}/ src)
aux_source_directory(${CMAKE_SOURCE_DIR}/com src)
aux_source_directory(${CMAKE_SOURCE_DIR}/robot src)
aux_source_directory(${CMAKE_SOURCE_DIR}/Task src)
aux_source_directory(${CMAKE_SOURCE_DIR}/vision src)

add_executable(${PROJECT_NAME} ${src} ${QRC_SOURCE_FILE})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/
                                                  ${CMAKE_SOURCE_DIR}/com
                                                  ${CMAKE_SOURCE_DIR}/robot
                                                  ${CMAKE_SOURCE_DIR}/Task
                                                  ${CMAKE_SOURCE_DIR}/vision
                                                  ${CMAKE_SOURCE_DIR}/ThirdParty/yaml-cpp-0.8.0/include/
                                                  ${CMAKE_SOURCE_DIR}/ThirdParty/cxxopts-3.2.0/include/
) 

# yaml-0.8.0
target_link_libraries(${PROJECT_NAME} yaml-cpp::yaml-cpp)

# qt5
target_link_libraries(${PROJECT_NAME}
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Network
        Qt5::Xml
)

# opencv + hk sdk
target_link_libraries(${PROJECT_NAME}
        opencv_world455d.lib
        MvCameraControl.lib
)

# ffmpeg
#target_link_libraries(${PROJECT_NAME}
#        avcodec.lib
#        avdevice.lib
#        avfilter.lib
#        avformat.lib
#        avutil.lib
#        postproc.lib
#        swresample.lib
#        swscale.lib
#)
# onnxruntime
target_link_libraries(${PROJECT_NAME}
        onnxruntime.dll
)

target_link_libraries(${PROJECT_NAME}
        BestfitLaserScanerSDK.lib
)


if (WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(DEBUG_SUFFIX)
    if (MSVC AND CMAKE_BUILD_TYPE MATCHES "Debug")
        set(DEBUG_SUFFIX "d")
    endif ()
    set(QT_INSTALL_PATH "${CMAKE_PREFIX_PATH}")
    if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
        set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        if (NOT EXISTS "${QT_INSTALL_PATH}/bin")
            set(QT_INSTALL_PATH "${QT_INSTALL_PATH}/..")
        endif ()
    endif ()
    if (EXISTS "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms/")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/plugins/platforms/qwindows${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>/plugins/platforms/")
    endif ()
    foreach (QT_LIB Core Gui Widgets)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy
                "${QT_INSTALL_PATH}/bin/Qt5${QT_LIB}${DEBUG_SUFFIX}.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
    endforeach (QT_LIB)
endif ()