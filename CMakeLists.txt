cmake_minimum_required(VERSION 3.0.2)
project(dt_ndt_mcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
)

find_package(Eigen3 REQUIRED)



catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} 
    "${PROJECT_SOURCE_DIR}/include"
)


add_executable(pf_node src/ndt_model.cpp src/pf_ros_node.cpp src/pf_ros src/scan.cpp src/scan_matcher_ndt.cpp)
target_link_libraries(pf_node ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


install(
  TARGETS
  pf_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)



