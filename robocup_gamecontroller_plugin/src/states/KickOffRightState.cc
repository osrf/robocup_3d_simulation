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
#include "robocup_gamecontroller_plugin/states/KickOffRightState.hh"
#include "robocup_gamecontroller_plugin/SoccerField.hh"

using namespace gazebo;

/////////////////////////////////////////////////
KickOffRightState::KickOffRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose1);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose2);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose3);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose4);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose5);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose6);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose7);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose8);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose9);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose10);
  this->leftInitialPoses.push_back(SoccerField::LeftInitPose11);

  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff1);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff2);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff3);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff4);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff5);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff6);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff7);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff8);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff9);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff10);
  this->rightInitialKickOffPoses.push_back(SoccerField::RightInitPoseKickOff11);
}

/////////////////////////////////////////////////
void KickOffRightState::Initialize()
{
  State::Initialize();

  this->plugin->ReleasePlayers();

  // Make sure the ball is at the center of the field
  this->plugin->MoveBall(math::Pose(0, 0, 0, 0, 0, 0));

  // Reposition the players
  for (size_t i = 0; i < this->plugin->teams.size(); ++i)
  {
    std::vector<math::Pose> initPoses;

    if (i == 0)
    {
      // Left team
      initPoses = this->leftInitialPoses;
    }
    else
    {
      // Right team
      initPoses = this->rightInitialKickOffPoses;
    }

    for (size_t j = 0; j < this->plugin->teams.at(i)->members.size(); ++j)
    {
      std::string name = this->plugin->teams.at(i)->members.at(j).second;
      physics::ModelPtr model = this->plugin->world->GetModel(name);
      if (model)
      {
        model->SetWorldPose(
          initPoses.at(this->plugin->teams.at(i)->members.at(j).first - 1));
      }
      else
        std::cerr << "Model (" << name << ") not found." << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void KickOffRightState::Update()
{
  this->plugin->StopPlayers();

  // After some time, go to play mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 2)
    this->plugin->SetCurrent(this->plugin->playState.get());
}
