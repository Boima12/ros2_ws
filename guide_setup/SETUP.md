Note: 
- Muon thuc hien duoc muc nay thi code branch main moi nhat phai co
- neu man hinh gazebo bi giat giat thi nhap lenh: export LIBGL_ALWAYS_SOFTWARE=1



# Cac lenh setup lan luot

<hr>
<br> <br> <br> <br> <br>

## I. Cai dat Ackermann va Setup node
> Cac task tuong ung: <br>
> Task: Tìm hiểu cách subscribe / publish trong ROS <br>
> Task: Tạo node ROS subscribe Odometry <br>
> Task: Node ROS publish điều khiển (cmd_vel / ackermann_cmd) <br>
>
> Tham khao: Node.docx cua Giang <br>

1. cai dat thu vien con thieu <br>

```
sudo apt update
sudo apt install ros-jazzy-ackermann-msgs
```

2. Build, source va chay node <br>

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

### ai bi trang man hinh trong terminal 2 thi: <br>

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

## II. Nap mo hinh xe ackermann va tao duong chay figure-8
> Cac task tuong ung: <br>
> Task: Nạp mô hình xe vào Gazebo <br>
> Task: Tạo đường chạy (oval / figure-8) <br>
>
> Tham khao: ackermann.docx cua Hoi <br>

1. chuan bi <br>
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

2. chay mo phong <br>
```
ros2 launch pure_pursuit_controller simulation.launch.py
```