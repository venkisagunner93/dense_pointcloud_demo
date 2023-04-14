cmake_minimum_required(VERSION 3.8)
project(dense_pointcloud_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pcl_pub_node src/pcl_pub_node.cpp)
add_executable(pcl_sub_node src/pcl_sub_node.cpp)
target_link_libraries(pcl_pub_node ${PCL_LIBRARIES})
target_link_libraries(pcl_sub_node ${PCL_LIBRARIES})
ament_target_dependencies(pcl_pub_node rclcpp std_msgs pcl_conversions)
ament_target_dependencies(pcl_sub_node rclcpp std_msgs pcl_conversions)

install(
  TARGETS
  pcl_pub_node
  pcl_sub_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
