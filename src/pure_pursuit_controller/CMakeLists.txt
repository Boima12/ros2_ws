cmake_minimum_required(VERSION 3.8)
project(pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm thư viện (Thêm geometry_msgs cho Twist)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # <--- BẮT BUỘC PHẢI CÓ
find_package(ament_index_cpp REQUIRED)

# 2. Tạo file thực thi
# (Không cần include_directories(src) nếu file .hpp nằm cạnh file .cpp)
add_executable(control_node src/control_node.cpp)

# 3. Link thư viện
ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs # <--- BẮT BUỘC PHẢI CÓ ĐỂ DÙNG TWIST
  ament_index_cpp
)

# 4. Cài đặt
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Cài đặt thư mục launch, urdf, worlds, src, config (để lệnh ros2 launch chạy được)
install(
  DIRECTORY launch urdf worlds src config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python test script
install(PROGRAMS
  src/test_steering.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
