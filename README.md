robocup_3d_simulation
=====================

![Alt text](/naos_robocup.png?raw=true "3D Simulation game simulated in Gazebo")

A repository for Gazebo and ROS based robocup_3d_simulation.

One-line installation
============

1. Execute the following line:

```
wget -O /tmp/robocup_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/robocup_install.sh; sudo sh /tmp/robocup_install.sh
```
Installation from sources
=========================

1. Install ROS. Follow instructions
 [here](http://wiki.ros.org/indigo/Installation/Ubuntu) and install the package
 `ros-indigo-desktop` (Ubuntu Trusty) or `ros-hydro-desktop` (Ubuntu Precise).

2. Install Gazebo4. Follow instructions
 [here](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) and
 install the branch `gazebo_4.1` or above. From now on we are going to assume
 that the installation is ROS indigo. If you are using hydro, just replace
 `indigo` with `hydro` in the following commands.

3. Install the Nao meshes:

    sudo apt-get install ros-indigo-nao-meshes

4. Clone the Robocup 3D simulation repository:

    cd $HOME

    git clone https://github.com/osrf/robocup_3d_simulation.git

2. Create a catkin workspace.

    . /opt/ros/indigo/setup.bash

    mkdir -p ~/robocup_ws/src && cd ~/robocup_ws/src

    ln -s ~/robocup_3d_simulation

    cd ..

    catkin_make

Running
=======

1. Start `roscore` in a new terminal:

    . /opt/ros/indigo/setup.bash

    roscore

2. Start Gazebo with the 3d simulation soccer field in a new terminal:

    . /opt/ros/indigo/setup.bash

    gazebo robocup3d.world

3. Spawn a team of agents:

    . /opt/ros/indigo/setup.bash

    spawnTeams

4. Run your own agent.:

    1. Run the s-expression interface program to be able to convert between ROS
    messages and s-expression. Open a new terminal and execute:

        . /opt/ros/indigo/setup.bash

        sExprInterface.py localhost 33001

    2. You can select your favorite port. Start and connect a real agent with the
    s-expression interface program in the port you entered in the previous step.
    For example:

        ./start.sh localhost -p 33001

[Optional] Additional agents can be inserted by typing:

    createAgent nao_soccer.sdf <team_name> <uniform_number>

5. [Optional] Run rcssserver3d just to draw the field in the roboviz:

    rcssserver3d

6. [Optional] Run roboviz to be able to debug the agents:

    roboviz.sh


Interacting with the simulation
===============================

We use ROS for interact with the simulation and mofify the state of the game.

