### Các câu lệnh Github

- Tạo branch mới để làm tính năng mới trên đó <br>
```
git branch new_branch
git checkout new_branch
```

- Đẩy branch mới lên Github <br>
```
git add .
git commit -m "Thêm chức năng gì đó"
git push origin new_branch
```

- cập nhật mới nhất từ GitHub <br>
```
git checkout main
git fetch --all --prune
git pull origin main
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

<br> <br> <br> <br> <br>
<hr>

### Cấu trúc thư mục của dự án
src/pure_pursuit_controller/package.xml - File định nghĩa thông tin package <br>
src/pure_pursuit_controller/CMakeLists.txt - File build của ROS2 sử dụng CMake <br>
src/pure_pursuit_controller/src/control_node.cpp - Đây là file code C++, là node điều khiển Pure Pursuit. <br>

<br> <br> <br> <br> <br>
<hr>

### Cách build và chạy node
- chạy node <br>
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 run pure_pursuit_controller control_node
```

- kiểm tra node hoạt động <br>
```
rqt_graph
```
