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
  
- **px4_connection:** Command analysis and other tools for communicate with px4 (both on sitl and onboard).
  
  - **msg_conversion:** Tools for running FUEL onboard, including keyboard control，sensor pose / control command conversion.

### others

- **bspline:** A implementation of the B-spline-based trajectory representation. 

- **bspline_opt:** The gradient-based trajectory optimization using B-spline trajectory.

- **poly_traj, traj_utils, utils**



## Quick Start

### Dependencies

This project has been tested on 18.04(ROS Melodic). Before you build it using `catkin_make`, make sure you meet the following requirements:

1. Basic ROS Environment (recommand desktop-full)

        sudo apt-get install ros-melodic-desktop-full

2. PX4

    Install PX4 using [v1.11.1](https://github.com/PX4/PX4-Autopilot/tree/v1.11.1):

        git clone -b 'v1.11.1' --single-branch --depth 1 https://github.com/PX4/PX4-Autopilot.git
        cd prometheus_px4
        git submodule update --init --recursive
        pip3 install --user toml empy jinja2 packaging
        source ./prometheus_px4/Tools/setup/ubuntu.sh
        make sitl_default gazebo

3. MavROS

        sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras

4. Gazebo Plugins
   
    For realsense sensors and lidar Gazebo plugins, in your workspace, run:

        git clone https://github.com/SeanZsya/gazebo_pulgins
        
5. OpenGL
        
        sudo apt-get install build-essential libgl1-mesa-dev
        sudo apt-get install freeglut3-dev
        sudo apt-get install libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev 

6. Other Required Tools 

        sudo apt-get install libarmadillo-dev ros-melodic-nlopt libdw-dev xmlstarlet

7. *Gazebo Models (optional)*

    Clone the third-party models:

        git clone https://github.com/osrf/gazebo_models
     
     Put them in `~/.gazebo/models/ `

### Path Setup

Add these lines to your ~./bashrc:

    source ${path to your workspace}/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${path to your workspace}/src/FUEL/fuel_in_gazebo/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:${path to your workspace}/devel/lib
    
### Run FUEL Simulation in Gazebo

**Launch Exploration:**

- For Solid-state lidar:

        roslaunch exploration_manager multi_exp_onboard_sslidar.launch

<!-- - For RGBD camera:

        roslaunch exploration_manager exploration_gazebo_rgbd.launch -->

Then follow the instructions inside the keyboard control tab, pressing `R` and using `2d Nav Goal` in Rviz to trigger exploration.