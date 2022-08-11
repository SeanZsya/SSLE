# FUEL (with SSLidar option and Gazebo Simulation)

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
  
- **run_onboard:** Tools for running FUEL onboard, including keyboard control，sensor pose / control command conversion.

### others

- **bspline:** A implementation of the B-spline-based trajectory representation. 

- **bspline_opt:** The gradient-based trajectory optimization using B-spline trajectory.

- **poly_traj, traj_utils, utils**



## Quick Start

### Dependencies

This project has been tested on 18.04(ROS Melodic). Before you build it using `catkin_make`, make sure you meet the following requirements:

1. Basic ROS Environment (recommand desktop-full)

        sudo apt install ros-melodic-desktop-full

2. PX4

    Coming soon.

3. Gazebo Plugins
   
    For realsense sensors and lidar Gazebo plugins, in your workspace, run:

        git clone https://github.com/SeanZsya/gazebo_pulgins

4. Other Required Tools 

        sudo apt-get install libarmadillo-dev ros-melodic-nlopt

5. *Gazebo Models (optional)*

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

        roslaunch exploration_manager exploration_gazebo_sslidar.launch

- For RGBD camera:

        roslaunch exploration_manager exploration_gazebo_rgbd.launch


**Start Keyboard Control:** (follow the instructions inside it)

    rosrun run_onboard pub_px4_cmd
