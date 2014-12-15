#!/bin/bash

# Copyright (C) 2012-2014 Open Source Robotics Foundation
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Description:
# This script installs Gazebo and the Robocup3D plugins onto an Ubuntu system.

codename=`lsb_release -sc`

# Make sure we are running a valid Ubuntu distribution
case $codename in
"precise")
  echo "You are running precise"
  ROSDISTRO="hydro"
  ;;
"trusty")
  echo "You are running trusty"
  ROSDISTRO="indigo"
  ;;
*)
  echo "This script will only work on Ubuntu precise or trusty"
  exit 0
esac

# Add the OSRF repository
if [ ! -e /etc/apt/sources.list.d/gazebo-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu ${codename} main\" > /etc/apt/sources.list.d/gazebo-latest.list"
fi

# Add the ROS repository
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${codename} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

# Download the OSRF keys
has_key=`apt-key list | grep "OSRF deb-builder"`

echo "Downloading keys"
if [ -z "$has_key" ]; then
  wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

# Update apt
echo "Retrieving packages"
sudo apt-get update -qq
echo "OK"

# Install gazebo and the Robocup3D plugins
echo "Installing packages"
sudo apt-get install --no-install-recommends ros-$ROSDISTRO-robocup-3d

echo "Complete."
