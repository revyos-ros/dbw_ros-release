#!/usr/bin/env bash

# To run at startup, add the line below to 'Startup Applications'
# gnome-terminal -e "/path/to/this/file/startup_joystick.bash"

# Source ROS source or binary workspace
if [ -e "$HOME/dbw_ws/install/setup.bash" ]; then
  source $HOME/dbw_ws/install/setup.bash
else
  source `find /opt/ros -name setup.bash | sort | head -1`
fi

# Launch
ros2 launch dbw_ford_joystick_demo joystick_demo.launch.xml sys:=true rviz:=false