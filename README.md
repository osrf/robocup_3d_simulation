robocup_3d_simulation
=====================

![Alt text](/naos_robocup.png?raw=true "3D Simulation game simulated in Gazebo")

A repository for Gazebo and ROS based robocup_3d_simulation.

Prerequisites
=============

We require Ubuntu Precise or Ubuntu Trusty.

One-line installation (recommended)
============

1. The following line will download a script that will install a package named
ros-{$DISTRO}-robocup3d in your machine. It will also install the required
dependencies, including the package containing the Nao meshes.

   ```
   wget -O /tmp/robocup3d_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/robocup3d_install.sh; sudo sh /tmp/robocup3d_install.sh
   ```

1. It is convenient if the environment variables are automatically added to your
bash session every time a new shell is launched.

  For ROS Hydro:

  ~~~
  echo -e "\n# Gazebo" >> ~/.bashrc
  echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
  echo "# ROS" >> ~/.bashrc
  echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ~~~

  For ROS Indigo:

  ~~~
  echo -e "\n# Gazebo" >> ~/.bashrc
  echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
  echo "# ROS" >> ~/.bashrc
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ~~~

Installation from sources
=========================

**You should skip this section if you installed the software using the one-line
instalation script.**

1. Install ROS. Follow instructions
 [here](http://wiki.ros.org/indigo/Installation/Ubuntu) and install the package
 `ros-indigo-desktop` (Ubuntu Trusty). If you are in Ubuntu Precise you should
 follow [this instructions](http://wiki.ros.org/hydro/Installation/Ubuntu) and
 install the package `ros-hydro-desktop`.

1. Install Gazebo4. Follow instructions
 [here](http://gazebosim.org/tutorials?tut=install_from_source&cat=install) and
 install the branch `gazebo_4.1` or above. From now on we are going to assume
 that the installation is ROS indigo. If you are using hydro, just replace
 `indigo` with `hydro` in the following commands.

1. It is convenient if the environment variables are automatically added to your
bash session every time a new shell is launched.

  For ROS Hydro:

  ~~~
  echo -e "\n# Gazebo" >> ~/.bashrc
  echo "source <REPLACE_BY_YOUR_GAZEBO_INSTALLATION_DIRECTORY>/gazebo/setup.sh" >> ~/.bashrc
  echo "# ROS" >> ~/.bashrc
  echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ~~~

  For ROS Indigo:

  ~~~
  echo -e "\n# Gazebo" >> ~/.bashrc
  echo "source <REPLACE_BY_YOUR_GAZEBO_INSTALLATION_DIRECTORY>/gazebo/setup.sh" >> ~/.bashrc
  echo "# ROS" >> ~/.bashrc
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ~~~

1. Install the Nao meshes:

  ~~~
  sudo apt-get install ros-indigo-nao-meshes
  ~~~

1. Clone the Robocup 3D simulation repository:

  ~~~
  cd $HOME
  git clone https://github.com/osrf/robocup_3d_simulation.git
  ~~~

1. Create a catkin workspace:

  ~~~
  mkdir -p ~/robocup_ws/src && cd ~/robocup_ws/src
  ln -s ~/robocup_3d_simulation
  cd ..
  catkin_make install
  ~~~

1. Prepare the environment:

  ~~~
  . install/setup.bash
  ~~~

Running
=======

1. Start `roscore` in a new terminal:

  ~~~
  roscore
  ~~~

1. Start Gazebo with the 3d simulation soccer field in a new terminal:

  ~~~
  gazebo robocup3d.world
  ~~~

1. Spawn a team of agents:

  ~~~
  spawnTeams
  ~~~

1. Run your own agent.:

    1. Run the s-expression interface program to be able to convert between ROS
    messages and s-expression. Open a new terminal and execute:

      ~~~
      sExprInterface.py localhost 33001
      ~~~

    1. You can select your favorite port. Start and connect a real agent with the
    s-expression interface program in the port you entered in the previous step.
    For example:

      ~~~
      ./start.sh localhost -p 33001
      ~~~

1. [Optional] Additional agents can be inserted by typing the following command,
  where the last two parameters are the team name and uniform number
  respectively:

  ~~~
  createAgent $ROS_ROOT/../robocup_model_resources/nao_models/nao_soccer.sdf teamA 3
  ~~~

1. [Optional] Run rcssserver3d just to draw the field in the roboviz:

  ~~~
  rcssserver3d
  ~~~

1. [Optional] Run roboviz to be able to debug the agents:

  ~~~
  roboviz.sh
  ~~~


Interacting with the simulation
===============================

We use ROS for interact with the simulation and modify the state of the game.
Spawn a team of agents following the instructions previously detailed.

1. Modify the state of the game. Open a new terminal and run:

  ~~~
  rosservice call /gameController/set_game_state KickOff_Left
  ~~~

  After a few seconds the game will switch to `PlayOn.

  The list of available game states is: `BeforeKickOff`, `KickOff_Left`,
  `KickOff_Right`, `PlayOn`, `KickIn_Left`, `KickIn_Right`, `corner_kick_left`,
  `corner_kick_right`, `goal_kick_left`, `goal_kick_right`, `GameOver`,
  `Goal_Left`, `Goal_Right`, `free_kick_left`, `kick_kick_right`.

1. Show the state of the game. Type in the previously open terminal:

  ~~~
  rostopic echo /gameController/game_state
  ~~~

1. Move the ball specifying [`<X>`, `<Y>`, `<Z>`, `<VX>`, `<VY>`, `<VZ>`]:

  ~~~
  rosservice call /gameController/move_ball 2 1 0 0 0
  ~~~

1. Move a player specifying [`<Team name>` `<uniform number>` [`<X>`, `<Y>`, `<THETA>`]:

  ~~~
  rosservice call /gameController/move_agent teamA 1 [3, 2, 0]
  ~~~
