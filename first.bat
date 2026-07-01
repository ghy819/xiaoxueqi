@echo off
chcp 65001

echo [INIT+TAG] ===== 初始化并创建 v1.0 =====

git init
git add .
git commit -m "v1.0 init version"

git branch -M main

set /p url=[INIT+TAG] 输入GitHub仓库地址:
git remote add origin %url%

git tag v1.0
git push -u origin main
git push origin v1.0

echo [INIT+TAG] ===== 完成 v1.0 =====
pause