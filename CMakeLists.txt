cmake_minimum_required(VERSION 2.8.3)
project(tue_calibration)

find_package(catkin REQUIRED COMPONENTS
	kdl_parser
	urdf
	tue_manipulation
    geometry_msgs
    message_generation

)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
#add_message_files(
#   FILES
#)

# Generate services
add_service_files(
   FILES
	getPose.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
   geometry_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(tue_cal_lib
    src/calibration.cpp
    include/tue_calibration/calibration.h
    src/chain.cpp
    include/tue_calibration/chain.h
    src/optimizer.cpp
    include/tue_calibration/optimizer.h
    src/marker_publisher.cpp
    include/tue_calibration/marker_publisher.h
)
target_link_libraries(tue_cal_lib ${catkin_LIBRARIES})

add_executable(calibration_node src/calibration_node.cpp)
target_link_libraries(calibration_node tue_cal_lib)


