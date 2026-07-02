@echo off
setlocal

echo ===== Git Auto Upload =====

call git add .
if errorlevel 1 goto error

set "msg="
set /p "msg=Commit message: "
if not defined msg (
    echo Commit message cannot be empty.
    goto end
)

call git commit -m "%msg%"
if errorlevel 1 goto error

call git push
if errorlevel 1 goto error

set "tag="
set /p "tag=Tag (example: v1.1, press Enter to skip): "
if not defined tag goto success

call git tag -a "%tag%" -m "%msg%"
if errorlevel 1 goto error

call git push origin "%tag%"
if errorlevel 1 goto error

:success
echo ===== Done =====
goto end

:error
echo ===== Failed - check the messages above =====

:end
pause
endlocal
