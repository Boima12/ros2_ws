Note: 
- Muốn thực hiện được mục này thì code branch main mới nhất phải có
- Nếu trong lúc làm màn hình gazebo bị giật thì xem chương III mục 3. Chạy Project phần # Cấu hình gazebo tránh bị giật


# Các lệnh setup lần lượt

<hr>
<br> <br> <br> <br> <br>

## I. Cài đặt Ackermann và Setup node
> Các task tương ứng: <br>
> Task: Tìm hiểu cách subscribe / publish trong ROS <br>
> Task: Tạo node ROS subscribe Odometry <br>
> Task: Node ROS publish điều khiển (cmd_vel / ackermann_cmd) <br>
>
> Tham khảo: Node.docx của Giang <br>

1. Cài đặt thư viện còn thiếu <br>

```
sudo apt update
sudo apt install ros-jazzy-ackermann-msgs
```

2. Build, source và chạy node <br>

```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

terminal 1: <br>
```
ros2 run pure_pursuit_controller control_node
```

terminal 2: <br>
```
rqt_graph
```
<br>

### Ai bị trắng màn hình trong terminal 2 thì: <br>

terminal 1: <br>
```
ros2 run pure_pursuit_controller control_node
```

terminal 2: <br>
```
ros2 topic pub /odom nav_msgs/msg/Odometry "{}"
```

terminal 3: <br>
```
rqt_graph
```

<hr>
<br> <br> <br> <br> <br>

## II. Nạp mô hình xe ackermann và tạo đường chạy figure-8
> Các task tương ứng: <br>
> Task: Nạp mô hình xe vào Gazebo <br>
> Task: Tạo đường chạy (oval / figure-8) <br>
>
> Tham khảo: ackermann.docx của Hội <br>

1. Chuẩn bị <br>
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

2. Chạy mô phỏng <br>
```
ros2 launch pure_pursuit_controller simulation.launch.py
```

<hr>
<br> <br> <br> <br> <br>

## III. Kết nối node Pure Pursuit vào waypoint path
> Cac task tuong ung: <br>
> Task: Kết nối node Pure Pursuit vào waypoint path <br>
> Task: Tuning tham số Lookahead để xe chạy ổn định <br>
>
> Tham khảo: node.docx của Giang, ackermann_car.docx của Hội, kết nối node và tuning.docx của Bảo <br>

0. Cập nhật và cài đặt dependencies bị thiếu(nếu có) <br>
```
sudo apt update
sudo apt install ros-jazzy-ackermann-msgs
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

1. Cập nhật mới từ Github <br>
```
cd ~/ros2_ws
git checkout main
git fetch --all --prune
git pull origin main
```

2. Build project <br>
```
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

3. Chạy Project <br>
```
# Cấu hình gazebo tránh bị giật (mỗi terminal chỉ cần chạy 1 lần)
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb
export OGRE_RTT_MODE=Copy

# Chạy chương trình
ros2 launch pure_pursuit_controller simulation.launch.py
```

<hr>
<br> <br> <br> <br> <br>