/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/State.hh"

using namespace gazebo;

/////////////////////////////////////////////////
State::State(const std::string &_name,
             GameControllerPlugin *_plugin)
  : name(_name), plugin(_plugin)
{
}

void State::Initialize()
{
  std::cout << "New state: " << this->name << std::endl;

  this->plugin->lastPlayerTouchedBall.first = -1;
  this->plugin->lastPlayerTouchedBall.second = "None";

  // Stop the ball
  if (this->plugin->ball)
  {
    this->plugin->ball->SetLinearVel(math::Vector3(0, 0, 0));
    this->plugin->ball->SetAngularVel(math::Vector3(0, 0, 0));
    this->plugin->ball->SetLinearAccel(math::Vector3(0, 0, 0));
    this->plugin->ball->SetAngularAccel(math::Vector3(0, 0, 0));
  }

  this->timer.Start();
}

/////////////////////////////////////////////////
std::string State::GetName()
{
  return this->name;
}
