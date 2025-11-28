cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tim thu vien
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# 2. Tao file thuc thi tu code C++
add_executable(control_node src/control_node.cpp)

# 3. Link thu vien vao file thuc thi
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  ackermann_msgs
)

# 4. Cai dat (De lenh ros2 run tim thay node)
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

