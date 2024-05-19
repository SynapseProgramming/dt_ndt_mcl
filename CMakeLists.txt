cmake_minimum_required(VERSION 3.8)
project(dt_ndt_mcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED) 


add_executable(dt_ndt_mcl_node src/pf_ros.cpp src/scan_matcher_ndt.cpp src/pf_ros_node.cpp src/particle_filter.cpp src/ndt_model.cpp src/motion_model.cpp src/scan.cpp)
ament_target_dependencies(dt_ndt_mcl_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_geometry_msgs tf2_ros tf2 angles Eigen3)
target_include_directories(dt_ndt_mcl_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  dt_ndt_mcl_node
  DESTINATION lib/${PROJECT_NAME})



ament_package()

