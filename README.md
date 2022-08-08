# FUEL (with SSLidar option and Gazebo Simulation)

## Project Structure


### Exploration

- **exploration_manager:** High-level modules that schedule and call the exploration algorithms. 

- **active_perception:** Find and organize frontier clusters, observe and avoid unknown obstacles.

### Planning

- **plan_manage:** High-level modules that schedule and call the mapping and planning algorithms. 

- **plan_env:** Online mapping algorithms, build an Euclidean signed distance filed (ESDF) for the planning systems 

- **bspline:** A implementation of the B-spline-based trajectory representation. 

- **bspline_opt:** The gradient-based trajectory optimization using B-spline trajectory.


poly_traj 

Traj_generator.cpp相对fast_planner无改动 

polynomial_traj.cpp删改较多 


 

工具类： 

traj_utils 

轨迹等元素的visualization 

Process_msg:  

未知作用的cloudCallback函数 

对点进行膨胀inflate 

uitls 

Lkh_tsp_solver：用来求解tsp问题


## Quick Start

### Dependencies

This project has been tested on 18.04(ROS Melodic).

1. Basic ROS Environment (recommand desktop-full)

        sudo apt install ros-melodic-desktop-full

2. PX4

    Coming soon.

3. Other Required Tools 

        sudo apt-get install libarmadillo-dev ros-melodic-nlopt

4. *Gazebo Models (optional)*

    In `~/.gazebo/models/ `, run:

        git clone https://github.com/osrf/gazebo_models

### Path Setup

Add these lines to your ~./bashrc:

    source <path to your workspace>/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to your workspace>/src/FUEL/fuel_in_gazebo/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<path to your workspace>/src/FUEL/fuel_in_gazebo/plugins
    
### Run FUEL Simulation in Gazebo

Launch Exploration:

    roslaunch exploration_manager exploration_gazebo_sslidar.launch

Start Keyboard Control (follow the instructions inside it):

    rosrun run_onboard pub_px4_cmd