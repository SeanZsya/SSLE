# FUEL (with SSLidar option and Gazebo Simulation)

## Quick Start

### Dependencies

This project has been tested on 18.04(ROS Melodic).

1. Basic ROS Environment (recommand desktop-full)

        sudo apt install ros-melodic-desktop-full

2. PX4

    Coming soon.

3. Baic Models

    In `~/.gazebo/models/ `, run:

        git clone https://github.com/osrf/gazebo_models

4. Other Required Tools 

        sudo apt-get install libarmadillo-dev ros-melodic-nlopt

### Path Setup

Add these lines to your ~./bashrc:

    source <path to your workspace>/devel/setup.bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to your workspace>/src/FUEL/fuel_in_gazebo/models
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:<path to your workspace>/src/FUEL/fuel_in_gazebo/plugins