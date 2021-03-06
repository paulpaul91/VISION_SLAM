cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -g -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(catkin REQUIRED COMPONENTS
 # roscpp
  #tf
  #sensor_msgs
  #std_msgs
  #geometry_msgs
#)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)


# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

rosbuild_add_executable(kitti_tf
src/ros_stereo_kitti.cc
)

target_link_libraries(kitti_tf
${LIBS}
)
# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(stereo_kitti
src/ros_stereo_pub_kitti.cc
)

target_link_libraries(stereo_kitti
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(stereo_kitti_tf
src/ros_stereo_pub_kitti_tf.cc
)

target_link_libraries(stereo_kitti_tf
${LIBS}
)
rosbuild_add_executable(RGBD11
src/ros_rgbdll.cc
)

target_link_libraries(RGBD11
${LIBS}
)

rosbuild_add_executable(ros_stereo_zed
src/ros_stereo_zed.cc
)

target_link_libraries(ros_stereo_zed
${LIBS}
)
rosbuild_add_executable(ros_stereo_zedxyyaw
src/ros_stereo_zedxyyaw.cc
)

target_link_libraries(ros_stereo_zedxyyaw
${LIBS}
)
