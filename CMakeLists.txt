cmake_minimum_required(VERSION 3.8)
project(car_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -Og)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include /opt/ros/humble/include/)

add_executable(point_cloud_to_scan src/point_cloud_to_scan.cpp)
ament_target_dependencies(point_cloud_to_scan rclcpp std_msgs sensor_msgs)


install(TARGETS
   point_cloud_to_scan
   DESTINATION lib/${PROJECT_NAME}
 )

ament_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_READ
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
