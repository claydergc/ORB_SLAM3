cmake_minimum_required(VERSION 2.4.6)
project(orb_slam3)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(ORB_SLAM3_DIR "/home/ros-noetic/src/ORB_SLAM3")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

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

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport std_msgs cv_bridge
  message_generation
)

add_message_files( FILES
  KeyPoint.msg
  KeyPointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${ORB_SLAM3_DIR}
${ORB_SLAM3_DIR}/include
${ORB_SLAM3_DIR}/include/CameraModels
${ORB_SLAM3_DIR}/Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deeper_introspective_slam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime std_msgs
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${catkin_LIBRARIES}
${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
-lboost_system
)



add_compile_options(-std=c++17)

# Node for monocular camera
#add_executable(Mono
#src/ros_mono.cc
#)

#target_link_libraries(Mono
#${LIBS}
#)

# Node for monocular camera (Augmented Reality Demo)
#add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)

#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
#add_executable(RGBD
#src/ros_rgbd.cc
#)

#target_link_libraries(RGBD
#${LIBS}
#)

# Node for monocular-inertial camera
#add_executable(Mono_Inertial
#src/ros_mono_inertial.cc
#)

#target_link_libraries(Mono_Inertial
#${LIBS}
#)

# Node for stereo-inertial camera
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBS})

add_executable(mono_inertial src/ros_mono_inertial.cc)
target_link_libraries(mono_inertial ${LIBS})

add_executable(Stereo_Inertial src/ros_stereo_inertial.cc)
target_link_libraries(Stereo_Inertial ${LIBS})
