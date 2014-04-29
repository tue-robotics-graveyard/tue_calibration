cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

file(GLOB_RECURSE HEADER_FILES include/*.h)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)



# Line detector
rosbuild_add_library(line_detector src/laser_detector.cpp)
rosbuild_add_executable(line_detector_node src/laser_detector_node.cpp ${HEADER_FILES})
target_link_libraries(line_detector_node line_detector)

# Kinect detector
rosbuild_add_executable(kinect_detector_node src/chessboard_detector_node.cpp src/detect_calibration_pattern.cpp ${HEADER_FILES})
#rosbuild_link_boost()
find_package(OpenCV)
target_link_libraries(kinect_detector_node ${OpenCV_LIBRARIES})

# Calibration node
rosbuild_add_library(calibration    src/calibration.cpp
                                    src/chain.cpp
                                    src/optimizer.cpp
                                    src/marker_publisher.cpp
)
rosbuild_add_executable(calibration_node src/calibration_node.cpp ${HEADER_FILES})
target_link_libraries(calibration_node calibration)
