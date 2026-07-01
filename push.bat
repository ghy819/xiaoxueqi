@echo off
echo ===== Git 自动上传 =====

git add .

set /p msg=请输入提交说明:
git commit -m "%msg%"

git push

echo ===== 完成 =====
pause