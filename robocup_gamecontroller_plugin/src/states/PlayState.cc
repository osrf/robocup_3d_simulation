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

#include <string>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/PlayState.hh"

using namespace gazebo;

/////////////////////////////////////////////////
PlayState::PlayState(const std::string &_name, GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void PlayState::Initialize()
{
  State::Initialize();

  this->plugin->ReleasePlayers();
  this->plugin->SetHalf(1);
  this->plugin->ResetClock();
}

/////////////////////////////////////////////////
void PlayState::Update()
{
  this->plugin->CheckTiming();
  this->plugin->CheckBall();
  this->plugin->CheckPlayerCollisions();
}
