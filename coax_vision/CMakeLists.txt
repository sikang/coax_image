cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(COAXHOME ${PROJECT_SOURCE_DIR}/../deploy)
set(COAXTARGET linux)
link_directories(${COAXHOME}/${COAXTARGET}/lib)
include_directories(${COAXHOME}/${COAXTARGET}/include)

include_directories(${PROJECT_SOURCE_DIR}/include)

SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
IF( NOT EIGEN3_INCLUDE_DIR )
MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#set(CMAKE_CXX_FLAGS,"-02 -msoft-float")
#rosbuild_add_executable(vfeedback src/vfeedback.cc)
#rosbuild_add_executable(readstate src/read_state.cc)
#rosbuild_add_executable(read_all src/read_all.cc)
#rosbuild_add_executable(extract_images src/extract_images.cc)
#rosbuild_add_executable(onboard_control 
#													src/onboard_control.cpp)
rosbuild_add_executable(img_listener src/img_listener.cpp)
rosbuild_add_executable(circle_detect src/circle_detect.cpp)
rosbuild_add_executable(contour_detect src/contour_detect.cpp)
rosbuild_add_executable(test_control 
													src/test_control.cpp
													src/VisionFeedback.cpp
													src/CoaxFilter.cpp)
#rosbuild_add_executable(coax_gumstix_control src/CoaxGumstixControl.cpp)
#rosbuild_add_executable(test_opencv src/test_opencv.cpp)
