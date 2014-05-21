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
#include <gazebo/physics/Model.hh>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/FreeKickLeftState.hh"

using namespace gazebo;

/////////////////////////////////////////////////
FreeKickRightState::FreeKickRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void FreeKickRightState::Initialize()
{
  State::Initialize();

  // Position the ball.
  if (this->plugin->ball)
    this->plugin->ball->SetWorldPose(math::Pose(this->pos.x,
      this->pos.y, this->pos.z, 0, 0, 0));
}

/////////////////////////////////////////////////
void FreeKickRightState::Update()
{
  // The left team is not allowed to be close to the ball.
  this->plugin->DropBallImpl(1);

  // After some time, go to play mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 5)
    this->plugin->SetCurrent(this->plugin->playState.get());
}

/////////////////////////////////////////////////
void FreeKickRightState::SetPos(const math::Vector3 &_pos)
{
  this->pos = _pos;
}
