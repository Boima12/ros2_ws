cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(pure_pursuit_node src/pure_pursuit_node.cpp)
ament_target_dependencies(pure_pursuit_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs ament_index_cpp)

install(TARGETS
  pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories (URDF, Worlds, Waypoints, Launch files)
install(DIRECTORY
  urdf
  worlds
  src
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()