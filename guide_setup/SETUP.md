Note: Muon thuc hien duoc muc nay thi code branch main moi nhat phai co




# Cac lenh setup lan luot

<hr>
<br> <br> <br>

## I. Cai dat Ackermann va 
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

