robocup_3d_simulation
=====================

A repository for Gazebo and ROS based robocup_3d_simulation.

Installation
============

1. Clone the repository:

    cd $HOME

    git clone https://github.com/osrf/robocup_3d_simulation.git

2. Create a catkin workspace.

    . /opt/ros/hydro/setup.bash
    
    mkdir -p ~/robocup_ws/src && cd ~/robocup_ws/src

    ln -s ../../robocup_3d_simulation
    
    cd ..
    
    catkin_make
    
Running
=======
    
1. Start roscore in a new terminal.

    . /opt/ros/hydro/setup.bash
    
    roscore

2. Start gazebo in a new terminal:

    . /opt/ros/hydro/setup.bash
     
    . ~/robocup_ws/devel/setup.bash

    gazebo ~/robocup_3d_simulation/worlds/robocup_3Dsim.world

3. Run a robocup client in a new terminal:
  
    . /opt/ros/hydro/setup.bash
     
    . ~/robocup_ws/devel/setup.bash

    ~/robocup_ws/devel/lib/robocup_clients/createAgent ~/robocup_3d_simulation/models/agent.sdf team1 1
