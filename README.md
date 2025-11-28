### Các câu lệnh Github

Tạo branch mới để làm tính năng mới trên đó

```
git branch new_branch
git checkout new_branch
```


Đẩy branch mới lên Github

```
git add .
git commit -m "Thêm chức năng gì đó"
git push origin new_branch
```


cập nhật mới nhất từ GitHub

```
git checkout main
git fetch --all --prune
git pull origin main
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
