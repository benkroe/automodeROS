#!/bin/bash
echo "Go to robostack"
cd ../robostack

echo "Activate virtual environment jazzy"
eval "$(pixi shell-hook -e jazzy)"

echo "Go back to workspace"
cd ../automode_ros_py
