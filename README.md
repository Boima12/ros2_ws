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

- cập nhật mới nhất từ GitHub vào branch main <br>
```
cd ~/ros2_ws
git checkout main
git fetch --all --prune
git pull origin main
```

<br> <br> <br> <br> <br>
<hr>

### Cấu trúc thư mục của dự án
src/pure_pursuit_controller/package.xml - File định nghĩa thông tin package, dependencies và metadata của ROS2 package <br>
src/pure_pursuit_controller/CMakeLists.txt - File build của ROS2 sử dụng CMake, cấu hình compile và install <br>
src/pure_pursuit_controller/launch/simulation.launch.py - File launch khởi động Gazebo simulator, spawn robot và các node điều khiển <br>
src/pure_pursuit_controller/src/pure_pursuit_node.cpp - Node chính thực hiện thuật toán Pure Pursuit để điều khiển xe theo waypoints <br>
src/pure_pursuit_controller/src/waypoints.txt - File chứa tọa độ các waypoints (x, y) để xe đi theo <br>
src/pure_pursuit_controller/urdf/ackermann_car.urdf - File mô tả cấu trúc robot dạng Ackermann (xe hơi)<br>
src/pure_pursuit_controller/worlds/oval_track.sdf - File world Gazebo đường đua hình số 8 với 2 chướng ngại vật hình trụ <br>
src/pure_pursuit_controller/worlds/line_track.sdf - File world Gazebo chứa môi trường đường đua hình chữ nhật <br> 

<br> <br> <br> <br> <br>
<hr>


