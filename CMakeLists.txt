#[===[

cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
]===]
# ^ this is the closing
cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Add executable
# Create obstacle_detection_node executable
add_executable(obstacle_detection_node
  src/obstacle_detection_node.cpp
)
target_link_libraries(obstacle_detection_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_filters_LIBRARIES}
  ${pcl_segmentation_LIBRARIES}
  ${pcl_features_LIBRARIES}
  ${pcl_visualization_LIBRARIES}
)
ament_target_dependencies(obstacle_detection_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  visualization_msgs
)
# Create point_cloud_merger_node executable
add_executable(point_cloud_merger_node
  src/PointCloudMergerNode.cpp
)
target_link_libraries(point_cloud_merger_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_filters_LIBRARIES}
  ${pcl_segmentation_LIBRARIES}
  ${pcl_features_LIBRARIES}
  ${pcl_visualization_LIBRARIES}
)
ament_target_dependencies(point_cloud_merger_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  visualization_msgs

)

# Install executable
install(TARGETS obstacle_detection_node point_cloud_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install package.xml
ament_package()
