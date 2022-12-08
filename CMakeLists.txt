cmake_minimum_required(VERSION 2.8.8)
project(ORB_SLAM2)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

#set(CMAKE_MODULE_PATH "/root/Desktop/Pangolin-0.5/build/src")

#message("-----------CMAKE_MODULE_PATH:${CMAKE_MODULE_PATH}")

find_package(OpenCV 3.2.0 EXACT REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR} #必须加这一行，应为有#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"这样的声明
        ${PROJECT_SOURCE_DIR}/include #必须加这一行，因为有#include "MapPoint.h"的声明
        ${Pangolin_INCLUDE_DIR}
        /usr/include/eigen3
)

message("-----------Pangolin_INCLUDE_DIR:${Pangolin_INCLUDE_DIR}")

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        )

message("-----------OpenCV_VERSION:${OpenCV_VERSION}")
message("-----------OpenCV_INSTALL_PATH:${OpenCV_INSTALL_PATH}")

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        )

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

add_executable(rgbd_igibson
        Examples/RGB-D/rgbd_igibson.cc)
target_link_libraries(rgbd_igibson ${PROJECT_NAME})
