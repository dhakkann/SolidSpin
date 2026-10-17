@echo off
title SolidSpin — SolidWorks Weapon Balancer
echo Launching SolidSpin — SolidWorks Weapon Balancer...
python run_balancer.py
if errorlevel 1 (
    echo.
    echo An error occurred running the Balancer.
    pause
)
