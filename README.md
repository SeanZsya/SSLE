# FUEL (with SSLidar option and Gazebo Simulation)

Adapted from [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL), an excellent drone exploration algorithm from Boyu Zhou.

## Project Structure


### Exploration

- **exploration_manager:** High-level modules that schedule and call the exploration algorithms. 

- **active_perception:** Find and organize frontier clusters, observe and avoid unknown obstacles.

### Planning

- **plan_manage:** High-level modules that schedule and call the mapping and planning algorithms. 

- **path_searching:** Path searching algorithems like kinodynamic astar and topology PRM.

### Mapping

- **plan_env:** Online mapping algorithms, build an Euclidean signed distance filed (ESDF) for the planning systems.

### Running Preparation

- **fuel_in_gazebo:** Components required by Gazebo, like models, worlds and pulgins.
  

### others

- **bspline:** A implementation of the B-spline-based trajectory representation. 

- **bspline_opt:** The gradient-based trajectory optimization using B-spline trajectory.

- **poly_traj, traj_utils, utils**



## Quick Start

### Dependencies

This project has been tested on 18.04(ROS Melodic). 

Before you build it using `catkin_make`, make sure you meet the following requirements:

1. Basic ROS Environment (recommand ros-melodic-desktop-full)

2. PX4

    Install PX4 using v1.11.1 (with configurations of drone model *Amov P450*):

        git clone -b 'v1.11.1-22.7.28' --single-branch --depth 1 https://gitee.com/amovlab/prometheus_px4.git
        cd prometheus_px4
        git submodule update --init --recursive
        source ./Tools/setup/ubuntu.sh --no-nuttx
        make px4_sitl_default gazebo

    If you still meet python dependencies (like `toml`, `jinja2`, etc.) problems after `source ./Tools/setup/ubuntu.sh` , try:

        cd ~/.local
        sudo chown -R ${username} lib/

3. PX4 connections

        git clone https://github.com/SeanZsya/basic_flight_suit

4. MavROS

        sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

5. Gazebo Plugins
   
    For realsense sensors and lidar Gazebo plugins, in your workspace, run:

        git clone https://github.com/SeanZsya/gazebo_pulgins
        
6. OpenGL
        
        sudo apt-get install build-essential libgl1-mesa-dev
        sudo apt-get install freeglut3-dev
        sudo apt-get install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev 

7. GeographicLib
        
        wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
        sudo ./install_geographiclib_datasets.sh

8. Other Required Tools 

        sudo apt-get install libarmadillo-dev ros-melodic-nlopt libdw-dev xmlstarlet

9.  *Gazebo Models (optional)*

    Clone the third-party models:

        git clone https://github.com/osrf/gazebo_models
     
     Put them in `~/.gazebo/models/ `

### Path Setup

Add these lines to your ~./bashrc:

    source ${path to your workspace}/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${path to your workspace}/src/FUEL/fuel_in_gazebo/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${path to your workspace}/devel/lib

    source ${path to your px4}/Tools/setup_gazebo.bash ${path to your px4} ${path to your px4}/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path to your px4}
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${path to your px4}/Tools/sitl_gazebo
    
### Run FUEL Simulation in Gazebo

**Launch Exploration:**

- For Solid-state lidar:

        roslaunch fuel_in_gazebo multi_exp_sim_sslidar.launch

<!-- - For RGBD camera:

        roslaunch exploration_manager exploration_gazebo_rgbd.launch -->

Then follow the instructions inside the keyboard control tab, pressing `R` and using `2d Nav Goal` in Rviz to trigger exploration.