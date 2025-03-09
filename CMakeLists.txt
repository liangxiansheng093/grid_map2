cmake_minimum_required(VERSION 3.5)
project(grid_map2)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find non-ROS libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

 add_executable(grid_map_node
         src/grid_map_node.cpp
         src/grid_map.cpp
         src/raycast.cpp
 )
 target_include_directories(grid_map_node PUBLIC
         include
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
         $<INSTALL_INTERFACE:include>
 )
 ament_target_dependencies(
         grid_map_node
         "rclcpp"
         "mavros_msgs"
         "PCL"
         "pcl_conversions"
         "visualization_msgs"
         "nav_msgs"
         "cv_bridge"
 )

add_library(grid_map2 SHARED
        src/grid_map.cpp
        src/raycast.cpp
        include/grid_map.h
        include/raycast.h
)
ament_target_dependencies(
        grid_map2
        "rclcpp"
        "mavros_msgs"
        "PCL"
        "pcl_conversions"
        "visualization_msgs"
        "nav_msgs"
        "cv_bridge"
)
ament_export_targets(grid_map2 HAS_LIBRARY_TARGET)
ament_export_dependencies(
        rclcpp
)
install(
        TARGETS grid_map2 # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT grid_map2
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        DIRECTORY include/
        DESTINATION include
)

 install(TARGETS grid_map_node
         DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
