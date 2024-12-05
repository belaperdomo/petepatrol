#!/bin/bash

export GPIOZERO_PIN_FACTORY=native
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo chmod -R o+rw /dev/input

cd ~/Desktop/robotics/Mee5650_ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/launch
ros2 launch pete_patrol_launch.yaml
