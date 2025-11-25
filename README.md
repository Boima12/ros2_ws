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
git restore .
git checkout main
git fetch --all --prune
git pull origin main
git branch -vv | grep ': gone]' | awk '{print $1}' | xargs git branch -D
```
