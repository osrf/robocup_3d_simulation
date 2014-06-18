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
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/BeforeKickOffState.hh"

using namespace gazebo;

/////////////////////////////////////////////////
BeforeKickOffState::BeforeKickOffState(const std::string &_name,
                                       GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
	// Left team initial positions during "before_kickoff" state.
  const std::string LPose1 = "<pose>-0.5 11 0 0 0 -1.57</pose>";
  const std::string LPose2 = "<pose>-1.5 11 0 0 0 -1.57</pose>";
  const std::string LPose3 = "<pose>-2.5 11 0 0 0 -1.57</pose>";
  const std::string LPose4 = "<pose>-3.5 11 0 0 0 -1.57</pose>";
  const std::string LPose5 = "<pose>-4.5 11 0 0 0 -1.57</pose>";
  const std::string LPose6 = "<pose>-5.5 11 0 0 0 -1.57</pose>";
  const std::string LPose7 = "<pose>-6.5 11 0 0 0 -1.57</pose>";
  const std::string LPose8 = "<pose>-7.5 11 0 0 0 -1.57</pose>";
  const std::string LPose9 = "<pose>-8.5 11 0 0 0 -1.57</pose>";
  const std::string LPose10 = "<pose>-9.5 11 0 0 0 -1.57</pose>";
  const std::string LPose11 = "<pose>-10.5 11 0 0 0 -1.57</pose>";

  // Right team initial positions during "before_kickoff" state.
  const std::string RPose1 = "<pose>0.5 11 0 0 0 -1.57</pose>";
  const std::string RPose2 = "<pose>1.5 11 0 0 0 -1.57</pose>";
  const std::string RPose3 = "<pose>2.5 11 0 0 0 -1.57</pose>";
  const std::string RPose4 = "<pose>3.5 11 0 0 0 -1.57</pose>";
  const std::string RPose5 = "<pose>4.5 11 0 0 0 -1.57</pose>";
  const std::string RPose6 = "<pose>5.5 11 0 0 0 -1.57</pose>";
  const std::string RPose7 = "<pose>6.5 11 0 0 0 -1.57</pose>";
  const std::string RPose8 = "<pose>7.5 11 0 0 0 -1.57</pose>";
  const std::string RPose9 = "<pose>8.5 11 0 0 0 -1.57</pose>";
  const std::string RPose10 = "<pose>9.5 11 0 0 0 -1.57</pose>";
  const std::string RPose11 = "<pose>10.5 11 0 0 0 -1.57</pose>";

	this->leftInitPoses.push_back(LPose1);
	this->leftInitPoses.push_back(LPose2);
	this->leftInitPoses.push_back(LPose3);
	this->leftInitPoses.push_back(LPose4);
	this->leftInitPoses.push_back(LPose5);
	this->leftInitPoses.push_back(LPose6);
	this->leftInitPoses.push_back(LPose7);
	this->leftInitPoses.push_back(LPose8);
	this->leftInitPoses.push_back(LPose9);
	this->leftInitPoses.push_back(LPose10);
	this->leftInitPoses.push_back(LPose11);

	this->rightInitPoses.push_back(RPose1);
	this->rightInitPoses.push_back(RPose2);
	this->rightInitPoses.push_back(RPose3);
	this->rightInitPoses.push_back(RPose4);
	this->rightInitPoses.push_back(RPose5);
	this->rightInitPoses.push_back(RPose6);
	this->rightInitPoses.push_back(RPose7);
	this->rightInitPoses.push_back(RPose8);
	this->rightInitPoses.push_back(RPose9);
	this->rightInitPoses.push_back(RPose10);
	this->rightInitPoses.push_back(RPose11);
}

/////////////////////////////////////////////////
void BeforeKickOffState::Initialize()
{
  State::Initialize();
  this->plugin->StopPlayers();
}

/////////////////////////////////////////////////
void BeforeKickOffState::Update()
{
  //this->plugin->StopPlayers();
}
