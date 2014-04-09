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

#include <gazebo/common/Time.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <algorithm>
#include <fstream>
#include <ros/ros.h>
#include <string>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_msgs/GameStateMonitor.h"
#include "robocup_msgs/InitAgent.h"

using namespace gazebo;

#define FIELD_WIDTH 20.0
#define FIELD_HEIGHT 30.0
#define TEAM_LEFT 0u
#define TEAM_RIGHT 1u
#define FREE_KICK_MOVE_DIST 15.15
#define FREE_KICK_DIST 9.15
#define GOAL_WIDTH 2.1

const math::Box FieldLeft(
    math::Vector3(-FIELD_HEIGHT*0.5, -FIELD_WIDTH*0.5, 0),
    math::Vector3(0, FIELD_WIDTH*0.5, 0));

const math::Box FieldRight(
    math::Vector3(0, -FIELD_WIDTH*0.5, 0),
    math::Vector3(FIELD_HEIGHT*0.5, FIELD_WIDTH*0.5, 0));

// Game state constant initialization
const std::string GameControllerPlugin::Kickoff  = "kickoff";
const std::string GameControllerPlugin::Playing  = "playing";
const std::string GameControllerPlugin::Finished = "finished";
const long GameControllerPlugin::SecondsEachHalf = 10000;

GZ_REGISTER_WORLD_PLUGIN(GameControllerPlugin)

/////////////////////////////////////////////////
const math::Pose InitPose1(math::Pose(-0.5, 0, 0, 0, 0, 0));
const math::Pose InitPose2(math::Pose(-0.5, -0.5, 0, 0, 0, 0));
const math::Pose InitPose3(math::Pose(-0.5, 0.5, 0, 0, 0, 0));
const math::Pose InitPose4(math::Pose(-FIELD_HEIGHT*0.5 + 0.5, 0, 0, 0, 0, 0));
std::vector<math::Pose> InitialPoses(4);

/////////////////////////////////////////////////
State::State(const std::string &_name,
             GameControllerPlugin *_plugin)
  : name(_name), plugin(_plugin)
{
}

/////////////////////////////////////////////////
std::string State::GetName()
{
  return this->name;
}

/////////////////////////////////////////////////
KickoffState::KickoffState(const std::string &_name,
                           GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void KickoffState::Initialize()
{
  // Make sure the ball is at the center of the field
  if (this->plugin->ball)
    this->plugin->ball->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 0));

  std::cout << "Num teams: " << this->plugin->teams.size() << std::endl;

  // Reposition the players
  for (size_t i = 0; i < this->plugin->teams.size(); ++i)
  {
    for (size_t j = 0; j < this->plugin->teams.at(i)->members.size(); ++j)
    {
      std::string name = this->plugin->teams.at(i)->members.at(j);
      physics::ModelPtr model = this->plugin->world->GetModel(name);
      if (model != NULL)
        model->SetWorldPose(InitialPoses.at(j));
      else
        std::cerr << "Model (" << name << ") not found." << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void KickoffState::Update()
{
}

/////////////////////////////////////////////////
PlayState::PlayState(const std::string &_name,
                     GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void PlayState::Initialize()
{
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

/////////////////////////////////////////////////
FinishedState::FinishedState(const std::string &_name,
                             GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void FinishedState::Initialize()
{
  this->plugin->StopClock();
}

/////////////////////////////////////////////////
void FinishedState::Update()
{
}

/////////////////////////////////////////////////
GameControllerPlugin::GameControllerPlugin()
{
  InitialPoses.push_back(InitPose1);
  InitialPoses.push_back(InitPose2);
  InitialPoses.push_back(InitPose3);
  InitialPoses.push_back(InitPose4);

  // Start up ROS
  std::string name = "gameController";
  int argc = 0;
  ros::init(argc, NULL, name);

  this->currentState = NULL;
  this->kickoffState = new KickoffState(this->Kickoff, this);
  this->playState = new PlayState(this->Playing, this);
  this->finishedState = new FinishedState(this->Finished, this);
  this->SetCurrent(this->finishedState);

  gzlog << "RoboCup 3D simulator game controller running" << std::endl;
}

/////////////////////////////////////////////////
GameControllerPlugin::~GameControllerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  delete this->kickoffState;
  this->kickoffState = NULL;
  delete playState;
  this->playState = NULL;
  delete this->finishedState;
  this->finishedState = NULL;
  this->currentState = NULL;

  delete this->node;
  this->node = NULL;
}

/////////////////////////////////////////////////
void GameControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, " <<
      "unable to load plugin. Load the Gazebo system plugin " <<
      "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // ROS Nodehandle
  this->node = new ros::NodeHandle("~");

  // Advertise all the services
  this->initAgentService = this->node->advertiseService("init_agent",
    &GameControllerPlugin::InitAgent, this);

  this->setGameStateService = this->node->advertiseService("set_game_state",
    &GameControllerPlugin::SetGameState, this);

  // Advertise all the messages
  this->publisher =
    this->node->advertise<robocup_msgs::GameStateMonitor>("game_state", 1000);

  this->world = _world;

  // Get a pointer to the soccer ball
  this->ball = this->world->GetModel(_sdf->Get<std::string>("ball"));

  if (!this->ball)
  {
    std::cerr << "Unable to find the soccer ball with name[" <<
      _sdf->Get<std::string>("ball") << "]\n";
    return;
  }

  /*
  // Load all the teams
  sdf::ElementPtr teamElem = _sdf->GetElement("team");
  while (teamElem)
  {
    // Create a new team
    Team *team = new Team;
    this->teams.push_back(team);

    // Set the team name
    team->name = teamElem->Get<std::string>("name");

    // Get all the team members.
    sdf::ElementPtr memberElem = teamElem->GetElement("member");
    while (memberElem)
    {
      // Get the team member name
      std::string memberName = memberElem->Get<std::string>();
      physics::ModelPtr member = this->world->GetModel(memberName);
      if (member)
        team->members.push_back(member);
      else
        gzerr << "Unable to get team member with name[" << memberName << "]\n";
      memberElem = memberElem->GetNextElement("member");
    }

    teamElem = teamElem->GetNextElement("team");
  }

  // Make sure that we have two teams.
  if (this->teams.size() != 2)
  {
    gzerr << "Invalid number of teams[" << this->teams.size()
      << "] for soccer.\n";
    return;
  }*/

  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GameControllerPlugin::UpdateStates, this, _1));
}

////////////////////////////////////////////////
uint8_t GameControllerPlugin::GetHalf()
{
  return this->half;
}

////////////////////////////////////////////////
void GameControllerPlugin::SetHalf(uint8_t _newHalf)
{
  if ((_newHalf == 1) || (_newHalf == 2))
    this->half = _newHalf;
  else
    gzerr << "Incorrect half value (" << _newHalf << ")" << std::endl;
}

////////////////////////////////////////////////
void GameControllerPlugin::Initialize()
{
  if (this->currentState)
    this->currentState->Initialize();
}

/////////////////////////////////////////////////
void GameControllerPlugin::Update()
{
  if (currentState)
    currentState->Update();
}

////////////////////////////////////////////////
void GameControllerPlugin::SetCurrent(State *_newState)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only update the state if _newState is different than the current state.
  if (this->currentState != _newState)
  {
    this->currentState = _newState;
    this->Initialize();
  }
}

bool GameControllerPlugin::InitAgent(
  robocup_msgs::InitAgent::Request  &req,
  robocup_msgs::InitAgent::Response &res)
{
  std::string agent = req.agent;
  std::string teamName = req.team_name;
  int player = req.player_number;

  std::cout << "New agent spawned" << std::endl;

  gzlog << "InitAgent called" << std::endl;
  gzlog << "\tAgent: " << agent << std::endl;
  gzlog << "\tTeam: " << teamName << std::endl;
  gzlog << "\tNumber: " << player << std::endl;

  // Create the team (if required)
  if ((this->teams.size() == 0) ||
      (this->teams.size() == 1) && this->teams.at(0)->name != teamName)
  {
    Team *aTeam = new Team;
    aTeam->name = teamName;
    this->teams.push_back(aTeam);
    std::cout << "New team" << std::endl;
  }

  // Chose your team
  Team *myTeam;
  if (teamName == this->teams.at(0)->name)
    myTeam = this->teams.at(0);
  else if (teamName == this->teams.at(1)->name)
    myTeam = this->teams.at(1);
  else
  {
    gzerr << "Incorrect team name (" << teamName << "). It seems that you already"
          << " registered two teams." << std::endl;
          return false;
  }

  std::ifstream myfile;
  std::string sdfContent = "";
  std::string line;

  myfile.open(agent.c_str());
  if (myfile.is_open())
  {
    while (getline(myfile, line))
    {
      sdfContent += line;
    }
    myfile.close();
  }
  else
  {
    std::cerr << "File (" << agent.c_str() << " not found" << std::endl;
    return false;
  }

  sdf::SDF agentSDF;
  agentSDF.SetFromString(sdfContent);

  std::string name;

  if (agentSDF.root->HasElement("model"))
  {
    sdf::ElementPtr modelElem = agentSDF.root->GetElement("model");
    if (modelElem->HasAttribute("name"))
    {
       name = modelElem->Get<std::string>("name");
       std::cout << "Agent name: (" << name << ")" << std::endl;
    }
  }

  this->world->InsertModelSDF(agentSDF);
  myTeam->members.push_back(name);

  res.result = 1;
  return true;
}

/////////////////////////////////////////////////
void GameControllerPlugin::ResetClock()
{
  this->startTimeSim = this->world->GetSimTime();
}

/////////////////////////////////////////////////
void GameControllerPlugin::StopClock()
{
  this->elapsedTimeSim = common::Time::Zero;
}

/////////////////////////////////////////////////
bool GameControllerPlugin::SetGameState(
  robocup_msgs::SetGameState::Request  &req,
  robocup_msgs::SetGameState::Response &res)
{
  if (req.play_mode == this->Playing)
    this->SetCurrent(this->playState);
  else if (req.play_mode == this->Kickoff)
    this->SetCurrent(this->kickoffState);
  else if (req.play_mode == this->Finished)
    this->SetCurrent(this->finishedState);
  else
  {
    gzerr << "[GameControllerPlugin::SetGameState()] Unknown play mode ("
          << req.play_mode << ")" << std::endl;
    res.result = 0;
    return false;
  }

  gzlog << "SetGameState called" << std::endl;
  gzlog << "\tPlay mode: " << this->currentState->GetName() << std::endl;

  res.result = 1;
  return true;
}

/////////////////////////////////////////////////
void GameControllerPlugin::Publish()
{
  robocup_msgs::GameStateMonitor msg;
  msg.time = ros::Time(this->elapsedTimeSim.Double());
  msg.half = this->GetHalf();
  msg.score_left = this->scoreLeft;
  msg.score_right = this->scoreRight;
  msg.play_mode = this->currentState->GetName();
  this->publisher.publish(msg);
}

/////////////////////////////////////////////////
void GameControllerPlugin::Init()
{
}

/////////////////////////////////////////////////
void GameControllerPlugin::UpdateStates(const common::UpdateInfo & /*_info*/)
{
  //this->teams[0]->members[0]->SetLinearVel(math::Vector3(1, 0, 0));

  this->Update();
  this->Publish();
  ros::spinOnce();
}

/////////////////////////////////////////////////
void GameControllerPlugin::CheckTiming()
{
  this->elapsedTimeSim = this->world->GetSimTime() - this->startTimeSim;

  if ((this->half == 1) && (this->elapsedTimeSim >= this->SecondsEachHalf))
  {
    // End of the first half
    this->SetHalf(2);
    std::swap(this->scoreLeft, this->scoreRight);
    this->SetCurrent(this->kickoffState);
    this->ResetClock();
    this->SetCurrent(this->playState);
  }
  else if ((this->GetHalf() == 2) && (elapsedTimeSim >= this->SecondsEachHalf))
  {
    // End of the game
    this->SetCurrent(this->finishedState);
  }
}

void GameControllerPlugin::CheckBall()
{
  // Get the position of the ball in the field reference frame.
  math::Pose ballPose = this->ball->GetWorldPose();

  if ((ballPose.pos.x < -FIELD_HEIGHT * 0.5) &&
      (fabs(ballPose.pos.y) < GOAL_WIDTH * 0.5))
  {
    // The ball is inside the left goal.
    this->scoreRight++;
    this->SetCurrent(this->kickoffState);
    this->SetCurrent(this->playState);
    gzlog << "Right team goal" << std::endl;
  }
  else if ((ballPose.pos.x > FIELD_HEIGHT * 0.5) &&
          (fabs(ballPose.pos.y) < GOAL_WIDTH * 0.5))
  {
    // The ball is inside the right goal.
    this->scoreLeft++;
    this->SetCurrent(this->kickoffState);
    this->SetCurrent(this->playState);
    gzlog << "Left team goal" << std::endl;
  }
  else if ((fabs(ballPose.pos.x) > FIELD_HEIGHT * 0.5) ||
      (fabs(ballPose.pos.y) > FIELD_WIDTH * 0.5))
  {
    // The ball is outside of the field.
    this->ball->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 0));
    gzlog << "Out of bounds" << std::endl;
  }
}

/////////////////////////////////////////////////
void GameControllerPlugin::CheckPlayerCollisions()
{
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    for (size_t j = 0; j < this->teams.at(i)->members.size() - 1; ++j)
    {
      std::string name = this->teams.at(i)->members.at(j);
      physics::ModelPtr model = this->world->GetModel(name);
      math::Pose pose = model->GetWorldPose();

      for (size_t k = j + 1; k < this->teams.at(i)->members.size(); ++k)
      {
        std::string otherName = this->teams.at(i)->members.at(k);
        physics::ModelPtr otherModel = this->world->GetModel(otherName);
        math::Pose otherPose = otherModel->GetWorldPose();

        std::cout << "Distance: " << pose.pos.Distance(otherPose.pos) << std::endl;
        if (pose.pos.Distance(otherPose.pos) < 0.20)
        {
          // Get the current pose of the member
          pose.pos.x = pose.pos.x -
              math::Rand::GetDblUniform(2, 2);

          /*
          // If the member is on the LEFT team, move the member's X position to
          // the LEFT
          if (_teamIndex == TEAM_LEFT)
          {
            pose.pos.x = _box.min.x -
              math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
          }
          else
          {
            pose.pos.x = _box.max.x +
              math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
          }*/

          model->SetWorldPose(pose);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
/*void GameControllerPlugin::ClearPlayers(const math::Box &_box, double _minDist,
    unsigned int _teamIndex)
{
  if (_teamIndex >= this->teams.size())
  {
    gzerr << "Invalid team index[" << _teamIndex << "]. "
      << "Max value is[" << this->teams.size() - 1 << "]\n";
    return;
  }

  for (std::vector<physics::ModelPtr>::iterator iter =
      this->teams[_teamIndex]->members.begin();
      iter != this->teams[_teamIndex]->members.end(); ++iter)
  {
    if ((*iter)->GetBoundingBox().Intersects(_box))
    {
      // Get the current pose of the member
      math::Pose pose = (*iter)->GetWorldPose();

      // If the member is on the LEFT team, move the member's X position to
      // the LEFT
      if (_teamIndex == TEAM_LEFT)
      {
        pose.pos.x = _box.min.x -
          math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
      }
      else
      {
        pose.pos.x = _box.max.x +
          math::Rand::GetDblUniform(_minDist, _minDist * 2.0);
      }

      (*iter)->SetWorldPose(pose);
    }
  }
}*/

/////////////////////////////////////////////////
/*void KickoffState::Init()
{
  // this->rules->SetPaused(true);
  // this->rules->ClearPlayers(FieldRight, FREE_KICK_MOVE_DIST, TEAM_LEFT);
  // this->rules->ClearPlayers(FieldLeft, FREE_KICK_MOVE_DIST, TEAM_RIGHT);
}*/
