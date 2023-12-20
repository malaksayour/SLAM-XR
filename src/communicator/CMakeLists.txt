cmake_minimum_required(VERSION 3.5)
project(communicator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

#find_package(downsampler_srv REQUIRED)
#find_package(align_srv REQUIRED)
find_package(custom_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
INCLUDE_DIRECTORIES(
  include
  ~/SLAM-XR/include
)



add_executable(com_node src/com_node.cpp)
ament_target_dependencies(com_node rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros pcl_conversions pcl_ros geometry_msgs custom_interfaces) 

# target_include_directories(com_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

install(TARGETS com_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(semantic_node src/semantic_node.cpp)
ament_target_dependencies(semantic_node rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros pcl_conversions pcl_ros geometry_msgs custom_interfaces) 

install(TARGETS semantic_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros pcl_conversions pcl_ros geometry_msgs nav_msgs custom_interfaces) 

install(TARGETS navigation_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(save_map_srv src/save_map.cpp)
ament_target_dependencies(save_map_srv rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros pcl_conversions pcl_ros geometry_msgs custom_interfaces) 

install(TARGETS save_map_srv
  DESTINATION lib/${PROJECT_NAME})

add_executable(load_map_srv src/load_map.cpp)
  ament_target_dependencies(load_map_srv rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros pcl_conversions pcl_ros geometry_msgs custom_interfaces) 
target_link_libraries(load_map_srv stdc++fs)

install(TARGETS load_map_srv
  DESTINATION lib/${PROJECT_NAME})

  add_executable(align_srv src/align.cpp)
  ament_target_dependencies(align_srv rclcpp std_msgs octomap_msgs sensor_msgs octomap_ros custom_interfaces pcl_conversions pcl_ros) 
  
  install(TARGETS align_srv
    DESTINATION lib/${PROJECT_NAME})

    add_executable(downsample_srv src/downsample.cpp)
    ament_target_dependencies(downsample_srv rclcpp std_msgs octomap_msgs sensor_msgs pcl_conversions pcl_ros octomap_ros custom_interfaces) 
    
    install(TARGETS downsample_srv
      DESTINATION lib/${PROJECT_NAME})

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
