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

    . ~/robocup_3d_simulation/models/nao_meshes/setup.sh (make sure that the
    paths match your directories, username, etc.).

    gazebo ~/robocup_3d_simulation/worlds/robocup_3Dsim.world

3. Manually spawn a team of agents:

    . /opt/ros/hydro/setup.bash

    . ~/robocup_ws/devel/setup.bash

    sh ~/robocup_3d_simulation/bin/spawnTeams.sh

[Optional] Additional agents can be inserted by typing:

    ~/robocup_ws/devel/lib/robocup_clients/createAgent ~/robocup_3d_simulation/models/agent.sdf <team_name> <uniform_number>

4. [Optional] Run rcssserver3d just to draw the field in the roboviz:

    rcssserver3d

5. [Optional] Run roboviz to be able to debug the agents:

    roboviz.sh

6. Run the s-expression interface program to be able to convert between ros
messages and s-expression.

    cd ~/robocup_3d_simulation/sExprInterface

    ./sExprInterface.py localhost 33001 (you can select your favourite port).

7. Start and connect a real agent with the s-expression interface program in
the port you entered in the previous step. For example:

    cd <directory_where_your_agent_is>
    ./start.sh -p 33001
