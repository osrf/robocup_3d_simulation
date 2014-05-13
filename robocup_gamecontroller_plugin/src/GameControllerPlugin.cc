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
#include <gazebo/common/Time.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_msgs/GameStateMonitor.h"
#include "robocup_msgs/InitAgent.h"

using namespace gazebo;

#define G_SQUARE(a) ( (a) * (a) )

#define FIELD_WIDTH 20.0
#define FIELD_HEIGHT 30.0
#define TEAM_LEFT 0u
#define TEAM_RIGHT 1u
#define FREE_KICK_MOVE_DIST 2
#define FREE_KICK_DIST 9.15
#define GOAL_WIDTH 2.1

const math::Box FieldLeft(
    math::Vector3(-FIELD_HEIGHT*0.5, -FIELD_WIDTH*0.5, 0),
    math::Vector3(0, FIELD_WIDTH*0.5, 0));

const math::Box FieldRight(
    math::Vector3(0, -FIELD_WIDTH*0.5, 0),
    math::Vector3(FIELD_HEIGHT*0.5, FIELD_WIDTH*0.5, 0));

// Game state constant initialization
const std::string GameControllerPlugin::BeforeKickOff   = "before_kickoff";
const std::string GameControllerPlugin::KickOffLeft     = "kickoff_left";
const std::string GameControllerPlugin::KickOffRight    = "kickoff_right";
const std::string GameControllerPlugin::Play            = "play";
const std::string GameControllerPlugin::KickInLeft      = "kickin_left";
const std::string GameControllerPlugin::KickInRight     = "kickin_right";
const std::string GameControllerPlugin::CornerKickLeft  = "corner_left";
const std::string GameControllerPlugin::CornerKickRight = "corner_right";
const std::string GameControllerPlugin::GoalKickLeft    = "goal_kick_left";
const std::string GameControllerPlugin::GoalKickRight   = "goal_kick_right";
const std::string GameControllerPlugin::GameOver        = "gameover";
const std::string GameControllerPlugin::GoalLeft        = "goal_left";
const std::string GameControllerPlugin::GoalRight       = "goal_right";
const std::string GameControllerPlugin::FreeKickLeft    = "free_kick_left";
const std::string GameControllerPlugin::FreeKickRight   = "kick_kick_right";

const long GameControllerPlugin::SecondsEachHalf = 10000;

GZ_REGISTER_WORLD_PLUGIN(GameControllerPlugin)

/////////////////////////////////////////////////
const math::Pose InitPoseKickOff1(math::Pose(-0.5, 0, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff2(math::Pose(-0.5, -0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff3(math::Pose(-0.5, 0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff4(math::Pose(-0.5, 0, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff5(math::Pose(-1.5, -0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff6(math::Pose(-1.5, 0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff7(math::Pose(-1.5, 0, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff8(math::Pose(-2.5, -0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff9(math::Pose(-2.5, 0.5, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff10(math::Pose(-2.5, 0, 0, 0, 0, 3.14));
const math::Pose InitPoseKickOff11(math::Pose(-FIELD_HEIGHT*0.5 + 0.5, 0, 0,
    0, 0, 3.14));

const math::Pose InitPose1(math::Pose(2.5, 0, 0, 0, 0, 0));
const math::Pose InitPose2(math::Pose(2.5, -0.5, 0, 0, 0, 0));
const math::Pose InitPose3(math::Pose(2.5, 0.5, 0, 0, 0, 0));
const math::Pose InitPose4(math::Pose(2.5, 0, 0, 0, 0, 0));
const math::Pose InitPose5(math::Pose(4.5, -0.5, 0, 0, 0, 0));
const math::Pose InitPose6(math::Pose(4.5, 0.5, 0, 0, 0, 0));
const math::Pose InitPose7(math::Pose(4.5, 0, 0, 0, 0, 0));
const math::Pose InitPose8(math::Pose(6.5, -0.5, 0, 0, 0, 0));
const math::Pose InitPose9(math::Pose(6.5, 0.5, 0, 0, 0, 0));
const math::Pose InitPose10(math::Pose(6.5, 0, 0, 0, 0, 0));
const math::Pose InitPose11(math::Pose(FIELD_HEIGHT*0.5 + 0.5, 0, 0,
    0, 0, 0));


/////////////////////////////////////////////////
//                STATES
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
BeforeKickOffState::BeforeKickOffState(const std::string &_name,
                                       GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void BeforeKickOffState::Initialize()
{
}

/////////////////////////////////////////////////
void BeforeKickOffState::Update()
{
}

/////////////////////////////////////////////////
KickOffLeftState::KickOffLeftState(const std::string &_name,
                                   GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void KickOffLeftState::Initialize()
{
  // Make sure the ball is at the center of the field
  if (this->plugin->ball)
    this->plugin->ball->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 0));

  std::cout << "Num teams: " << this->plugin->teams.size() << std::endl;

  // Reposition the players
  for (size_t i = 0; i < this->plugin->teams.size(); ++i)
  {
    std::vector<math::Pose> initPoses;

    if (i == 0)
    {
      // Left team
      initPoses = this->plugin->initialKickOffPoses;
    }
    else
    {
      // Right team
      initPoses = this->plugin->initialPoses;
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
void KickOffLeftState::Update()
{
}

/////////////////////////////////////////////////
KickOffRightState::KickOffRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void KickOffRightState::Initialize()
{
}

/////////////////////////////////////////////////
void KickOffRightState::Update()
{
}

/////////////////////////////////////////////////
PlayState::PlayState(const std::string &_name, GameControllerPlugin *_plugin)
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
KickInLeftState::KickInLeftState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void KickInLeftState::Initialize()
{
}

/////////////////////////////////////////////////
void KickInLeftState::Update()
{
}

/////////////////////////////////////////////////
KickInRightState::KickInRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void KickInRightState::Initialize()
{
}

/////////////////////////////////////////////////
void KickInRightState::Update()
{
}

/////////////////////////////////////////////////
CornerKickLeftState::CornerKickLeftState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void CornerKickLeftState::Initialize()
{
}

/////////////////////////////////////////////////
void CornerKickLeftState::Update()
{
}

/////////////////////////////////////////////////
CornerKickRightState::CornerKickRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void CornerKickRightState::Initialize()
{
}

/////////////////////////////////////////////////
void CornerKickRightState::Update()
{
}

/////////////////////////////////////////////////
GoalKickLeftState::GoalKickLeftState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void GoalKickLeftState::Initialize()
{
}

/////////////////////////////////////////////////
void GoalKickLeftState::Update()
{
}

/////////////////////////////////////////////////
GoalKickRightState::GoalKickRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void GoalKickRightState::Initialize()
{
}

/////////////////////////////////////////////////
void GoalKickRightState::Update()
{
}

/////////////////////////////////////////////////
GameOverStateState::GameOverStateState(const std::string &_name,
                                       GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void GameOverStateState::Initialize()
{
  this->plugin->StopClock();
}

/////////////////////////////////////////////////
void GameOverStateState::Update()
{
}

/////////////////////////////////////////////////
GoalLeftState::GoalLeftState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void GoalLeftState::Initialize()
{
}

/////////////////////////////////////////////////
void GoalLeftState::Update()
{
}

/////////////////////////////////////////////////
GoalRightState::GoalRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void GoalRightState::Initialize()
{
}

/////////////////////////////////////////////////
void GoalRightState::Update()
{
}

/////////////////////////////////////////////////
FreeKickLeftState::FreeKickLeftState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void FreeKickLeftState::Initialize()
{
}

/////////////////////////////////////////////////
void FreeKickLeftState::Update()
{
}

/////////////////////////////////////////////////
FreeKickRightState::FreeKickRightState(const std::string &_name,
                         GameControllerPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void FreeKickRightState::Initialize()
{
}

/////////////////////////////////////////////////
void FreeKickRightState::Update()
{
}

/////////////////////////////////////////////////
//                 PLUGIN
/////////////////////////////////////////////////
GameControllerPlugin::GameControllerPlugin()
  : beforeKickOffState(new BeforeKickOffState(this->BeforeKickOff, this)),
    kickOffLeftState(new KickOffLeftState(this->KickOffLeft, this)),
    kickOffRightState(new KickOffRightState(this->KickOffRight, this)),
    playState(new PlayState(this->Play, this)),
    kickInLeftState(new KickInLeftState(this->KickInLeft, this)),
    kickInRightState(new KickInRightState(this->KickInRight, this)),
    cornerKickLeftState(new CornerKickLeftState(this->CornerKickLeft, this)),
    cornerKickRightState(new CornerKickRightState(this->CornerKickRight, this)),
    goalKickLeftState(new GoalKickLeftState(this->GoalKickLeft, this)),
    goalKickRightState(new GoalKickRightState(this->GoalKickRight, this)),
    gameOverState(new GameOverStateState(this->GameOver, this)),
    goalLeftState(new GoalLeftState(this->GoalLeft, this)),
    goalRightState(new GoalRightState(this->GoalRight, this)),
    freeKickLeftState(new FreeKickLeftState(this->FreeKickLeft, this)),
    freeKickRightState(new FreeKickRightState(this->FreeKickRight, this))
{
  initialPoses.push_back(InitPose1);
  initialPoses.push_back(InitPose2);
  initialPoses.push_back(InitPose3);
  initialPoses.push_back(InitPose4);
  initialPoses.push_back(InitPose5);
  initialPoses.push_back(InitPose6);
  initialPoses.push_back(InitPose7);
  initialPoses.push_back(InitPose8);
  initialPoses.push_back(InitPose9);
  initialPoses.push_back(InitPose10);
  initialPoses.push_back(InitPose11);

  initialKickOffPoses.push_back(InitPoseKickOff1);
  initialKickOffPoses.push_back(InitPoseKickOff2);
  initialKickOffPoses.push_back(InitPoseKickOff3);
  initialKickOffPoses.push_back(InitPoseKickOff4);
  initialKickOffPoses.push_back(InitPoseKickOff5);
  initialKickOffPoses.push_back(InitPoseKickOff6);
  initialKickOffPoses.push_back(InitPoseKickOff7);
  initialKickOffPoses.push_back(InitPoseKickOff8);
  initialKickOffPoses.push_back(InitPoseKickOff9);
  initialKickOffPoses.push_back(InitPoseKickOff10);
  initialKickOffPoses.push_back(InitPoseKickOff11);

  // Start up ROS
  std::string name = "gameController";
  int argc = 0;
  ros::init(argc, NULL, name);

  this->SetCurrent(this->beforeKickOffState);

  gzlog << "RoboCup 3D simulator game controller running" << std::endl;
}

/////////////////////////////////////////////////
GameControllerPlugin::~GameControllerPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
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

  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->requestPub = this->gzNode->Advertise<msgs::Request>("~/request");

  // ROS Nodehandle
  this->node.reset(new ros::NodeHandle("~"));

  // Advertise all the services
  this->initAgentService = this->node->advertiseService("init_agent",
    &GameControllerPlugin::InitAgent, this);

  this->setGameStateService = this->node->advertiseService("set_game_state",
    &GameControllerPlugin::SetGameState, this);

  this->moveAgentService = this->node->advertiseService("move_agent",
    &GameControllerPlugin::MoveAgentPose, this);

  this->moveBallService = this->node->advertiseService("move_ball",
    &GameControllerPlugin::MoveBall, this);

  this->dropBallService = this->node->advertiseService("drop_ball",
    &GameControllerPlugin::DropBall, this);

  this->killAgentService = this->node->advertiseService("kill_agent",
    &GameControllerPlugin::KillAgent, this);

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
void GameControllerPlugin::SetCurrent(const StatePtr &_newState)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Only update the state if _newState is different than the current state.
  if (this->currentState.get() != _newState.get())
  {
    this->currentState = _newState;
    this->Initialize();
  }
}

/////////////////////////////////////////////////
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

  // Check the player id is correct.
  if (player <= 0 || player > 11)
  {
    gzerr << "Incorrect player #(" << player << ")" << std::endl;
    return false;
  }

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
    gzerr << "Incorrect team name (" << teamName << "). It seems that you "
          << "already registered two teams." << std::endl;
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

  // Add the player
  // ToDo(caguero): Check that the player is not already existing.
  myTeam->members.push_back(make_pair(player, name));

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
  if (req.play_mode == this->BeforeKickOff)
    this->SetCurrent(this->beforeKickOffState);
  else if (req.play_mode == this->KickOffLeft)
    this->SetCurrent(this->kickOffLeftState);
  else if (req.play_mode == this->KickOffRight)
    this->SetCurrent(this->kickOffRightState);
  else if (req.play_mode == this->Play)
    this->SetCurrent(this->playState);
  else if (req.play_mode == this->KickInLeft)
    this->SetCurrent(this->kickInLeftState);
  else if (req.play_mode == this->KickInRight)
    this->SetCurrent(this->kickInRightState);
  else if (req.play_mode == this->CornerKickLeft)
    this->SetCurrent(this->cornerKickLeftState);
  else if (req.play_mode == this->CornerKickRight)
    this->SetCurrent(this->cornerKickRightState);
  else if (req.play_mode == this->GoalKickLeft)
    this->SetCurrent(this->goalKickLeftState);
  else if (req.play_mode == this->GoalKickRight)
    this->SetCurrent(this->goalKickRightState);
  else if (req.play_mode == this->GameOver)
    this->SetCurrent(this->gameOverState);
  else if (req.play_mode == this->GoalLeft)
    this->SetCurrent(this->goalLeftState);
  else if (req.play_mode == this->GoalRight)
    this->SetCurrent(this->goalRightState);
  else if (req.play_mode == this->FreeKickLeft)
    this->SetCurrent(this->freeKickLeftState);
  else if (req.play_mode == this->FreeKickRight)
    this->SetCurrent(this->freeKickRightState);
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
bool GameControllerPlugin::MoveAgentPose(
  robocup_msgs::MoveAgentPose::Request  &req,
  robocup_msgs::MoveAgentPose::Response &res)
{
  // Find the team
  int index = -1;
  for (size_t i = 0; i < this->teams.size(); ++i)
    if (this->teams.at(i)->name == req.team_name)
    {
      index = i;
      break;
    }

  // Team not found
  if (index == -1)
  {
    gzlog << "Trying to move an agent from an unknown team ("
          << req.team_name << ")" << std::endl;
    return false;
  }

  // Wrong player #
  Members_It it = std::find_if(this->teams.at(index)->members.begin(),
                    this->teams.at(index)->members.end(),
                    CompareFirst(req.player_id));

  if (it == this->teams.at(index)->members.end())
  //if (req.player_id <= 0 || req.player_id > 11)
  {
    gzlog << "Trying to move an agent with an unregistered id ("
          << req.player_id << ")" << std::endl;
    return false;
  }

  // Move the player
  std::string name = it->second;
  physics::ModelPtr model = this->world->GetModel(name);
  if (!model)
  {
    std::cerr << "MoveAgentPose(). Model (" << name << ") not found.\n";
    return false;
  }

  math::Pose newPose(math::Pose(req.position.x, req.position.y, 0,
                                0, 0, req.position.theta));
  model->SetWorldPose(newPose);
  return true;
}

/////////////////////////////////////////////////
bool GameControllerPlugin::MoveBall(
  robocup_msgs::MoveBall::Request  &req,
  robocup_msgs::MoveBall::Response &res)
{
  physics::ModelPtr model = this->world->GetModel("soccer_ball");
  if (model != NULL)
  {
    math::Pose newPose(math::Pose(req.x, req.y, req.z, 0, 0, 0));
    math::Vector3 newVel(req.vx, req.vy, req.vz);
    model->SetWorldPose(newPose);
    model->SetLinearVel(newVel);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
bool GameControllerPlugin::DropBall(robocup_msgs::DropBall::Request  &req,
                                    robocup_msgs::DropBall::Response &res)
{
  res.result = 0;

  // Get ball position.
  physics::ModelPtr model = this->world->GetModel("soccer_ball");
  if (!model)
  {
    std::cerr << "DropBall() error: Ball not found" << std::endl;
    return false;
  }

  math::Vector3 ballPos = model->GetWorldPose().pos;

  std::cout << "Teams: " << this->teams.size() << std::endl;

  // Check if the player is withing FREE_KICK distance.
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    std::cout << this->teams.at(i)->members.size() << std::endl;
    for (size_t j = 0; j < this->teams.at(i)->members.size(); ++j)
    {
      std::string name = this->teams.at(i)->members.at(j).second;
      model = this->world->GetModel(name);

      if (model)
      {
        std::cout << "Model not null" << std::endl;
        math::Pose playerPose = model->GetWorldPose();

        // Move the player if it's close enough to the ball.
        if (playerPose.pos.Distance(ballPos) < FREE_KICK_MOVE_DIST)
        {

          std::cout << "Player close to the ball" << std::endl;
          // Calculate the general form equation of a line from two points.
          // a = y1 - y2
          // b = x2 - x1
          // c = (x1-x2)*y1 + (y2-y1)*x1
          math::Vector3 v(ballPos.y - playerPose.pos.y,
                          playerPose.pos.x - ballPos.x,
                          (ballPos.x - playerPose.pos.x) * ballPos.y +
                          (playerPose.pos.y - ballPos.y) * ballPos.x);
          math::Vector3 int1;
          math::Vector3 int2;
          if (this->IntersectionCircunferenceLine(v, ballPos,
                                                  FREE_KICK_MOVE_DIST,
                                                  int1, int2))
          {
            if (playerPose.pos.Distance(int1) < playerPose.pos.Distance(int2))
              playerPose.pos = int1;
            else
              playerPose.pos = int2;

            model->SetWorldPose(playerPose);
          }
          else
            gzerr << "DropBall() error: No intersection between circunference"
                  << " and line. That shouldn't be happening" << std::endl;
        }
        else
          std::cout << "Player not close to the ball" << std::endl;

      }
      else
        std::cout << "Model is NULL" << std::endl;
    }
  }

  res.result = 1;
  return true;
}

/////////////////////////////////////////////////
bool GameControllerPlugin::KillAgent(robocup_msgs::KillAgent::Request  &req,
                                     robocup_msgs::KillAgent::Response &res)
{
  res.result = 0;

  // Find the team
  int index = -1;
  for (size_t i = 0; i < this->teams.size(); ++i)
    if (this->teams.at(i)->name == req.team_name)
    {
      index = i;
      break;
    }

  // Team not found
  if (index == -1)
  {
    gzerr << "Trying to kill an agent from an unknown team ("
          << req.team_name << ")" << std::endl;
    return false;
  }

  // Wrong player #
  Members_It it = std::find_if(this->teams.at(index)->members.begin(),
                    this->teams.at(index)->members.end(),
                    CompareFirst(req.player_number));

  if (it == this->teams.at(index)->members.end())
  {
    gzerr << "Trying to kill an agent with an unregistered id ("
          << req.player_number << ")" << std::endl;
    return false;
  }

  // Kill the player
  std::string name = it->second;
  physics::ModelPtr model = this->world->GetModel(name);
  if (!model)
  {
    gzerr << "KillAgent(). Model (" << name << ") not found.\n";
    return false;
  }

  msgs::Request *msg = msgs::CreateRequest("entity_delete", name);
  this->requestPub->Publish(*msg);

  // Remove the player from the map.
  this->teams.at(index)->members.erase(
      std::remove_if(this->teams.at(index)->members.begin(),
                     this->teams.at(index)->members.end(),
                     CompareFirst(req.player_number)),
      this->teams.at(index)->members.end());
  //this->teams.at(index)->members.erase(req.player_number);
  //this->teams.at(index)->members.at(req.player_number) = "";

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
  // this->teams[0]->members[0]->SetLinearVel(math::Vector3(1, 0, 0));

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
    this->SetCurrent(this->kickOffRightState);
    this->ResetClock();
    this->SetCurrent(this->playState);
  }
  else if ((this->GetHalf() == 2) && (elapsedTimeSim >= this->SecondsEachHalf))
  {
    // End of the game
    this->SetCurrent(this->gameOverState);
  }
}

/////////////////////////////////////////////////
void GameControllerPlugin::CheckBall()
{
  // Get the position of the ball in the field reference frame.
  math::Pose ballPose = this->ball->GetWorldPose();

  if ((ballPose.pos.x < -FIELD_HEIGHT * 0.5) &&
      (fabs(ballPose.pos.y) < GOAL_WIDTH * 0.5))
  {
    // The ball is inside the left goal.
    this->scoreRight++;
    //this->SetCurrent(this->kickoffState);
    this->SetCurrent(this->playState);
    gzlog << "Right team goal" << std::endl;
  }
  else if ((ballPose.pos.x > FIELD_HEIGHT * 0.5) &&
          (fabs(ballPose.pos.y) < GOAL_WIDTH * 0.5))
  {
    // The ball is inside the right goal.
    this->scoreLeft++;
    //this->SetCurrent(this->kickoffState);
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
// ToDo(caguero): Fix this method.
void GameControllerPlugin::CheckPlayerCollisions()
{
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    for (size_t j = 0; j < this->teams.at(i)->members.size() - 1; ++j)
    {
      std::string name = this->teams.at(i)->members.at(j).second;
      physics::ModelPtr model = this->world->GetModel(name);

      if (model)
      {
        math::Pose pose = model->GetWorldPose();

        for (size_t k = j + 1; k < this->teams.at(i)->members.size(); ++k)
        {
          std::string otherName = this->teams.at(i)->members.at(k).second;
          physics::ModelPtr otherModel = this->world->GetModel(otherName);

          if (otherModel)
          {
            math::Pose otherPose = otherModel->GetWorldPose();

            //std::cout << "Distance: " << pose.pos.Distance(otherPose.pos)
            //          << std::endl;
            if (pose.pos.Distance(otherPose.pos) < 0.40)
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

bool GameControllerPlugin::IntersectionCircunferenceLine(const math::Vector3 &_v,
                                                       const math::Vector3 &p_c,
                                                       float r,
                                                       math::Vector3 &int1,
                                                       math::Vector3 &int2)
{
  // Solve equations:
  // (x-px)^2 + (y - py)^2 = r^2
  // Ax + By + C = 0

  float i, j, k, A_2;
  float a, b, c;
  float tmp;
  math::Vector3 v;

  v = _v;

  if(v.x == 0.0) {
    // Avoid div by 0
    v.x = 0.000001;
  }

  i = -2 * p_c.x;
  j = -2 * p_c.y;
  k = G_SQUARE(p_c.x) + G_SQUARE(p_c.y) - G_SQUARE(r);

  A_2 = G_SQUARE(v.x);
  a = G_SQUARE(-v.y) / A_2 + 1;
  b = -2 * v.z * -v.y / A_2  - v.y * i / v.x + j;
  c = G_SQUARE(v.z) / A_2 - v.z * i / v.x + k;

  // Solve a*Y^2 + b+Y + c = 0
  tmp = G_SQUARE(b) - 4 * a * c;
  if(tmp < 0) {
    // No intersection
    return false;
  }

  tmp = sqrt(tmp);
  int1.y = (-b + tmp) / (2 * a);
  int2.y = (-b - tmp) / (2 * a);

  int1.x = (-v.y * int1.y - v.z) / v.x;
  int2.x = (-v.y * int2.y - v.z) / v.x;
}
