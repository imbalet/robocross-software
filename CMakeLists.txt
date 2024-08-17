cmake_minimum_required(VERSION 3.8)
project(car_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g -O0)
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
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(include /opt/ros/humble/include/ /usr/include/eigen3 /usr/include/opencv4/)

add_executable(pc2scan src/pc2scan.cpp)
ament_target_dependencies(pc2scan rclcpp std_msgs sensor_msgs)

add_executable(path_mapping src/path_mapping.cpp)
ament_target_dependencies(
  path_mapping 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_ros)

target_link_libraries(path_mapping ${OpenCV_LIBS})

install(TARGETS
   pc2scan
   path_mapping
   DESTINATION lib/${PROJECT_NAME}
 )



# add_executable(imu src/imu.cpp)
# ament_target_dependencies(imu rclcpp sensor_msgs std_msgs)

# install(TARGETS
#  imu
#  DESTINATION lib/${PROJECT_NAME}
# )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/cloud_to_scan.py
  scripts/camera_driver.py
  scripts/pathfinding.py
  scripts/path_controller.py
  scripts/path_mapping.py
  scripts/utils.py
  scripts/pc2scan.py
  scripts/waypoint_following.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
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
