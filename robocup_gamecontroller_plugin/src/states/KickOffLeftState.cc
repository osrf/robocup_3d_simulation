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
#include "robocup_gamecontroller_plugin/states/KickOffLeftState.hh"
#include "robocup_gamecontroller_plugin/SoccerField.hh"

using namespace gazebo;

/////////////////////////////////////////////////
KickOffLeftState::KickOffLeftState(const std::string &_name,
                                   GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff1);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff2);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff3);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff4);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff5);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff6);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff7);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff8);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff9);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff10);
  this->leftInitialKickOffPoses.push_back(SoccerField::LeftInitPoseKickOff11);

  this->rightInitialPoses.push_back(SoccerField::RightInitPose1);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose2);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose3);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose4);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose5);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose6);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose7);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose8);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose9);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose10);
  this->rightInitialPoses.push_back(SoccerField::RightInitPose11);
}

/////////////////////////////////////////////////
void KickOffLeftState::Initialize()
{
  State::Initialize();

  this->plugin->ReleasePlayers();

  // Make sure the ball is at the center of the field
  this->plugin->MoveBall(math::Pose(0, 0, 0, 0, 0, 0));

  // Reposition the players
  for (size_t i = 0; i < this->plugin->teams.size(); ++i)
  {
    std::vector<math::Pose> initPoses;

    // Left team
    if (i == 0)
    {
      initPoses = this->leftInitialKickOffPoses;
    }
    // Right team
    else
    {
      initPoses = this->rightInitialPoses;
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

  this->plugin->StopPlayers();
}

/////////////////////////////////////////////////
void KickOffLeftState::Update()
{

  // After some time, go to play mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 2)
    this->plugin->SetCurrent(this->plugin->playState.get());
}
