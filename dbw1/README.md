# dbw_ros/dbw1
ROS2 interface to Dataspeed drive-by-wire platforms with legacy DBW1 firmware

# Launch

* Joystick demo
    * DBW1 FCA:     `ros2 launch dbw_fca_joystick_demo joystick_demo.launch.xml sys:=true`
    * DBW1 Ford:    `ros2 launch dbw_ford_joystick_demo joystick_demo.launch.xml sys:=true`
    * DBW1 Polaris: `ros2 launch dbw_polaris_joystick_demo joystick_demo.launch.xml sys:=true`
* Drive-by-wire only
    * DBW1 FCA:     `ros2 launch dbw_fca_can dbw.launch.xml`
    * DBW1 Ford:    `ros2 launch dbw_ford_can dbw.launch.xml`
    * DBW1 Polaris: `ros2 launch dbw_polaris_can dbw.launch.xml`
* RViz visualization
    * DBW1 FCA:     `ros2 launch dbw_fca_description rviz.launch.xml`
    * DBW1 Ford:    `ros2 launch dbw_ford_description rviz.launch.xml`
    * DBW1 Polaris: `ros2 launch dbw_polaris_description rviz.launch.xml`

# Binaries

* ROS buildfarm with infrequent updates:
    * http://repo.ros2.org/status_page/ros_humble_default.html?q=dbw_ros
* Dataspeed buildfarm with frequent updates:
    * https://bitbucket.org/DataspeedInc/ros_binaries/

# One Line Install (binary)

* Use this option to install ROS2 package binaries on a workstation that already has ROS2 installed.
* Paste the following into a terminal to install the binary packages. This script will configure apt-get to connect to the Dataspeed server and install the binary packages.
* FCA:     ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_fca/scripts/sdk_install.bash)```
* Ford:    ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_ford/scripts/sdk_install.bash)```
* Polaris: ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_polaris/scripts/sdk_install.bash)```

# One Line ROS2 and Packages Install (binary)

* Use this option to install ROS2 and this SDK on a clean Ubuntu install.
* This should ONLY be run on a fresh install of Ubuntu Desktop [22.04](http://releases.ubuntu.com/22.04/)/[20.04](http://releases.ubuntu.com/20.04/).
* Paste the following into a terminal to install ROS2 and this SDK. This script will change some operating system parameters, install ROS2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)/[Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html), install the SDK, and configure the joystick demo with rviz to run at startup.
* FCA:     ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_fca/scripts/ros_install.bash)```
* Ford:    ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_ford/scripts/ros_install.bash)```
* Polaris: ```bash <(wget -q -O - https://bitbucket.org/DataspeedInc/dbw_ros/raw/ros2/dbw1/dbw_polaris/scripts/ros_install.bash)```
