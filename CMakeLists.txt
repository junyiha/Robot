cmake_minimum_required(VERSION 3.28)
project(PDRobot-master)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "D:/Qt514/5.14.0/msvc2017_64")
set(BaseDIR D:/)
#set(CMAKE_PREFIX_PATH "D:/soft/qt5.14/5.14.0/msvc2017_64")
#set(BaseDIR E:/znzz/codes/qt/PDRootV1)

# �������о��棨���Ƽ���
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")

# ���߽����ض�����
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /C4828")
set(QRC_SOURCE_FILE res.qrc)

# ffmpeg
file(COPY ${BaseDIR}/packages/ffmpeg/bin/avcodec-59.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${BaseDIR}/packages/ffmpeg/bin/avformat-59.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${BaseDIR}/packages/ffmpeg/bin/avformat-59.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${BaseDIR}/packages/ffmpeg/bin/swresample-4.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)

# opencv
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455d.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${BaseDIR}/packages/opencv4.55/build/x64/vc15/bin/opencv_world455.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)

# Qt
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Core.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Cored.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Gui.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Guid.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgets.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/bin/Qt5Widgetsd.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug)
file(COPY ${CMAKE_PREFIX_PATH}/plugins/platforms DESTINATION ${CMAKE_BINARY_DIR}/Debug/plugins/)

# HK
file(COPY ${BaseDIR}/packages/HK_SDK/Libraries/win64/MvCameraControl.lib DESTINATION ${CMAKE_BINARY_DIR}/Debug/)

# GoSdk
file(COPY ${BaseDIR}/packages/GocatorSDK/bin/win64/kApi.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)
file(COPY ${BaseDIR}/packages/GocatorSDK/bin/win64/GoSdk.dll DESTINATION ${CMAKE_BINARY_DIR}/Debug/)

# Opencv Setting
include_directories(${BaseDIR}/packages/opencv4.55/build/include)
link_directories(${BaseDIR}/packages/opencv4.55/build/x64/vc15/lib)

# Spdlog Setting
include_directories(${BaseDIR}/packages)

#HK SDK  Setting
include_directories(${BaseDIR}/packages/HK_SDK/Includes)
link_directories(${BaseDIR}/packages/HK_SDK/Libraries/win64)

# ffmpeg setting
include_directories(${BaseDIR}/packages/ffmpeg/include)
link_directories(${BaseDIR}/packages/ffmpeg/lib)

# Gocator setting
include_directories(${BaseDIR}/packages/GocatorSDK/include)
link_directories(${BaseDIR}/packages/GocatorSDK/lib/win64d)


# onnxruntime setting
include_directories(${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.1.7.0/build/native/include)
link_directories(${BaseDIR}/packages/OnnxRuntime/Microsoft.ML.OnnxRuntime.1.7.0/runtimes/win-x64/native)

# eigen3
include_directories(${BaseDIR}/packages/eigen3.4.0)


#find_package(Qt5 COMPONENTS Core Widgets Xml REQUIRED)
##set(Resources_RCC ${CMAKE_CURRENT_BINARY_DIR}/Resources.rcc)
#qt5_add_resources(${Resources_RCC} res.qrc)

#add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
#add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

add_compile_options("$<$<C_COMPILER_ID:MSVC>:/source-charset:utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")

find_package(Qt5 COMPONENTS
        Core
        Gui
        Widgets
        Network
        Xml
        REQUIRED)

#  add resources
qt5_add_resources(${QRC_SOURCE_FILE})

add_executable(PDRobot-master main.cpp
        com/ComInterface.cpp
        com/ComInterface.h
        com/IOCom.cpp
        com/IOCom.h
        com/Manual.cpp
        com/Manual.h
        com/RobotCom.cpp
        com/RobotCom.h
        com/SerialCom.cpp
        com/SerialCom.h
        com/TcpCom.cpp
        com/TcpCom.h
        com/TcpCom_IO.cpp
        com/TcpCom_IO.h
        com/Tools.cpp
        com/Tools.h
        robot/Axis.h
        robot/DataStruct.h
        robot/ErrCode.h
        robot/GeneralRobot.h
        robot/Link.h
        robot/LinkObject.h
        robot/Parameter.h
        robot/robot.cpp
        robot/robot.h
        robot/RobotKinectModel.cpp
        robot/RobotKinectModel.h
        robot/RobotModuleServices.h
        robot/Transformation.cpp
        robot/Transformation.h
        Task/Measure.cpp
        Task/Measure.h
        Task/Task.cpp
        Task/Task.h
        Task/TaskParameter.h
        GVL.h
        vision/VisionControls.cpp
        vision/VisionControls.h
        vision/CameraDisplay.cpp
        vision/CameraDisplay.h
        vision/Camera.cpp
        vision/Camera.h
        vision/CameraControls.cpp
        vision/CameraControls.h
        vision/FFmpegCamera.cpp
        vision/FFmpegCamera.h
        vision/Gocator.cpp
        vision/Gocator.h
        vision/GocatorControls.cpp
        vision/GocatorControls.h
        vision/HoleHandler.cpp
        vision/HoleHandler.h
        vision/HoleHelper.cpp
        vision/HoleHelper.h
        vision/LidarHandler.cpp
        vision/LidarHandler.h
        vision/LidarHelper.cpp
        vision/LidarHelper.h
        vision/LineHandler.cpp
        vision/LineHandler.h
        vision/LineHelper.cpp
        vision/LineHelper.h
        vision/mlsd.cpp
        vision/mlsd.h
        vision/myequeue.h
        vision/utils.cpp
        vision/utils.h
        vision/ShareData.h
        vision/CameraManager.cpp
        vision/CameraManager.h
        vision/HKCameraControls.cpp
        vision/HKCameraControls.h
        vision/MvCamera.cpp
        vision/MvCamera.h
        vision/Parameters.h
        ${QRC_SOURCE_FILE}
        vision/LineDetector.cpp
        vision/LineDetector.h
        vision/VisionInterface.cpp
        vision/VisionInterface.h
        vision/LineDetectorRunner.cpp
        vision/LineDetectorRunner.h
        mainwindow.cpp
        mainwindow.h
        #        vision/VisionTask.cpp
#        vision/VisionTask.h
#        vision/CameraShow.cpp
#        vision/CameraShow.h
)
target_link_libraries(PDRobot-master
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::Network
        Qt5::Xml
)

# opencv + hk sdk
target_link_libraries(PDRobot-master
        opencv_world455d.lib
        MvCameraControl.lib
)

# ffmpeg
target_link_libraries(PDRobot-master
        avcodec.lib
        avdevice.lib
        avfilter.lib
        avformat.lib
        avutil.lib
        postproc.lib
        swresample.lib
        swscale.lib
)
# onnxruntime
target_link_libraries(PDRobot-master onnxruntime.dll)

target_link_libraries(PDRobot-master
        GoSdk.lib
        kApi.lib
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

