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

#ifndef _GAZEBO_GAME_CONTROLLER_PLUGIN_HH_
#define _GAZEBO_GAME_CONTROLLER_PLUGIN_HH_

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <map>
#include <string>
#include <vector>
#include "robocup_gamecontroller_plugin/states/BeforeKickOffState.hh"
#include "robocup_gamecontroller_plugin/states/CornerKickLeftState.hh"
#include "robocup_gamecontroller_plugin/states/CornerKickRightState.hh"
#include "robocup_gamecontroller_plugin/states/FreeKickLeftState.hh"
#include "robocup_gamecontroller_plugin/states/FreeKickRightState.hh"
#include "robocup_gamecontroller_plugin/states/GameOverState.hh"
#include "robocup_gamecontroller_plugin/states/GoalKickLeftState.hh"
#include "robocup_gamecontroller_plugin/states/GoalKickRightState.hh"
#include "robocup_gamecontroller_plugin/states/GoalLeftState.hh"
#include "robocup_gamecontroller_plugin/states/GoalRightState.hh"
#include "robocup_gamecontroller_plugin/states/KickInLeftState.hh"
#include "robocup_gamecontroller_plugin/states/KickInRightState.hh"
#include "robocup_gamecontroller_plugin/states/KickOffLeftState.hh"
#include "robocup_gamecontroller_plugin/states/KickOffRightState.hh"
#include "robocup_gamecontroller_plugin/states/PlayState.hh"
#include "robocup_msgs/DropBall.h"
#include "robocup_msgs/InitAgent.h"
#include "robocup_msgs/KillAgent.h"
#include "robocup_msgs/SetGameState.h"
#include "robocup_msgs/MoveAgentPose.h"
#include "robocup_msgs/MoveBall.h"

#define TEAM_LEFT 0u
#define TEAM_RIGHT 1u

namespace gazebo
{
  typedef std::vector<std::pair<int, std::string> > Members_V;
  typedef Members_V::iterator Members_It;
  typedef boost::shared_ptr<State> StatePtr;

  class GameControllerPlugin : public WorldPlugin
  {
    public: static const std::string BeforeKickOff;
    public: static const std::string KickOffLeft;
    public: static const std::string KickOffRight;
    public: static const std::string Play;
    public: static const std::string KickInLeft;
    public: static const std::string KickInRight;
    public: static const std::string CornerKickLeft;
    public: static const std::string CornerKickRight;
    public: static const std::string GoalKickLeft;
    public: static const std::string GoalKickRight;
    public: static const std::string GameOver;
    public: static const std::string GoalLeft;
    public: static const std::string GoalRight;
    public: static const std::string FreeKickLeft;
    public: static const std::string FreeKickRight;
    public: static const long        SecondsEachHalf;

    /// \brief Constructor.
    public: GameControllerPlugin();

    /// \brief Destructor.
    public: virtual ~GameControllerPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Check if the first half or the game ends.
    public: void CheckTiming();

    /// \brief Check the ball's position looking for goals or out of bounds.
    public: void CheckBall();

    public: void CheckPlayerCollisions();

    /// \brief Reset the internal soccer game clock.
    public: void ResetClock();

    /// \brief Stop the internal soccer game clock.
    public: void StopClock();

    /// \brief Get the game's half.
    /// \return 1 if the game is in the first half or 2 if is in the second.
    public: uint8_t GetHalf();

    /// \brief Set the game half.
    /// \param[in] _newHalf 1 for first half or 2 for second half.
    public: void SetHalf(uint8_t _newHalf);

    /// \brief ROS service callback that spawns an agent into the simulation.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call result.
    /// \return True when the service call succeeds.
    private: bool InitAgent(robocup_msgs::InitAgent::Request  &_req,
                            robocup_msgs::InitAgent::Response &_res);

    /// \brief Method executed each time the game state changes.
    private: void Initialize();

    /// \brief Publish all the ROS messages.
    private: void Publish();

    /// \brief Set the current game state. If the new state is the same than
    /// the current one, the operation does not have any effect.
    /// \param [out] _newState
    public: void SetCurrent(State *_newState);

    /// \brief ROS service callback that sets the state of the game.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool SetGameState(robocup_msgs::SetGameState::Request  &req,
                               robocup_msgs::SetGameState::Response &res);

    /// \brief ROS service callback that move an agent.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool MoveAgentPose(robocup_msgs::MoveAgentPose::Request  &req,
                                robocup_msgs::MoveAgentPose::Response &res);

    /// \brief ROS service callback that move the ball.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool MoveBall(robocup_msgs::MoveBall::Request  &req,
                           robocup_msgs::MoveBall::Response &res);

    /// \brief ROS service callback that drops the ball at its current position
    /// and move all players away by the free kick radius. If the ball is off
    /// the field, it is brought back within bounds.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool DropBall(robocup_msgs::DropBall::Request  &req,
                           robocup_msgs::DropBall::Response &res);

    /// \briedf Drops the ball at its current positionand move all players away
    /// by the free kick radius. If the ball is off the field, it is brought
    /// back within bounds.
    /// \param[in] _teamAllowed 0 if the left team is allowed to be close to the
    /// ball. 1 if the right team is allowed or any other number if none of the
    // teams are allowed to be within the free kick radius.
    public: bool DropBallImpl(const int _teamAllowed);

    /// \brief ROS service callback that removes the specified agent from
    /// the simulation.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool KillAgent(robocup_msgs::KillAgent::Request  &req,
                            robocup_msgs::KillAgent::Response &res);

    /// \brief Executes the update on the current state.
    private: void Update();

    /// \brief Update the robocup simulation state.
    /// \param[in] _info Information used in the update event.
    private: void UpdateStates(const common::UpdateInfo &_info);

    private: bool IntersectionCircunferenceLine(const math::Vector3 &v,
                                                const math::Vector3 &p_c,
                                                float r,
                                                math::Vector3 &int1,
                                                math::Vector3 &int2);

    private: void OnBallContacts(ConstContactsPtr &_msg);

    public: void ReleasePlayers();

    public: void StopPlayers();

    public: void MoveBall(const math::Pose &_pose);

    public: math::Pose GetBall();

    /// \brief Pointer to the world.
    public: physics::WorldPtr world;

    /// \brief Pointer to the soccer ball.
    public: physics::ModelPtr ball;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    /// \brief Request publisher.
    private: transport::PublisherPtr requestPub;

    // ROS Node handler
    private: boost::scoped_ptr<ros::NodeHandle> node;

    // Gazebo subscription to contact messages on the ball.
    private: transport::SubscriberPtr ballSub;

    // ROS Service for spawning new agents.
    private: ros::ServiceServer initAgentService;

    // ROS Service for change the state of the game.
    private: ros::ServiceServer setGameStateService;

    // ROS Service for move a player.
    private: ros::ServiceServer moveAgentService;

    // ROS Service for move the ball.
    private: ros::ServiceServer moveBallService;

    // ROS Service for dropping the ball.
    private: ros::ServiceServer dropBallService;

    // ROS Service for killing an agent.
    private: ros::ServiceServer killAgentService;

    // ROS Publisher.
    private: ros::Publisher publisher;

    /// \brief Game half (1st half or 2nd half).
    private: uint8_t half;

    /// \brief Left team score.
    public: uint32_t scoreLeft;

    /// \brief Right team score.
    public: uint32_t scoreRight;

    /// \brief Sim time at which the game started.
    private: gazebo::common::Time startTimeSim;

    /// \brief Pointer to the current game state.
    private: State *currentState;

    /// \brief Pointer to the kickoff state.
    public: boost::shared_ptr<BeforeKickOffState> beforeKickOffState;

    /// \brief Pointer to the kickoffLeft state.
    public: boost::shared_ptr<KickOffLeftState> kickOffLeftState;

    /// \brief Pointer to the kickoffRight state.
    public: boost::shared_ptr<KickOffRightState> kickOffRightState;

    /// \brief Pointer to the play state.
    public: boost::shared_ptr<PlayState> playState;

    /// \brief Pointer to the kickInLeft state.
    public: boost::shared_ptr<KickInLeftState> kickInLeftState;

    /// \brief Pointer to the kickInRight state.
    public: boost::shared_ptr<KickInRightState> kickInRightState;

    /// \brief Pointer to the cornerKickLeft state.
    public: boost::shared_ptr<CornerKickLeftState> cornerKickLeftState;

    /// \brief Pointer to the cornerKickRight state.
    public: boost::shared_ptr<CornerKickRightState> cornerKickRightState;

    /// \brief Pointer to the goalKickLeft state.
    public: boost::shared_ptr<GoalKickLeftState> goalKickLeftState;

    /// \brief Pointer to the goalKickRight state.
    public: boost::shared_ptr<GoalKickRightState> goalKickRightState;

    /// \brief Pointer to the gameover state.
    public: boost::shared_ptr<GameOverState> gameOverState;

    /// \brief Pointer to the goalLeft state.
    public: boost::shared_ptr<GoalLeftState> goalLeftState;

    /// \brief Pointer to the goalRight state.
    public: boost::shared_ptr<GoalRightState> goalRightState;

    /// \brief Pointer to the freeKickLeft state.
    public: boost::shared_ptr<FreeKickLeftState> freeKickLeftState;

    /// \brief Pointer to the freeKickRight state.
    public: boost::shared_ptr<FreeKickRightState> freeKickRightState;

    /// \brief Game time.
    private: common::Time elapsedTimeSim;

    /// \brief Mutex to avoid race conditions while running updates and a ROS
    /// callback is executed.
    private: boost::mutex mutex;

    /// \brief (left or right, player_name).
    public: std::pair<int, std::string> lastPlayerTouchedBall;

    struct CompareFirst
    {
      CompareFirst(int val) : val(val) {}
      bool operator()(const std::pair<int, std::string>& _elem) const {
        return val == _elem.first;
      }
      private:
        int val;
    };

    /// \brief A single team.
    public: class Team
    {
      /// \brief Name of the team.
      public: std::string name;

      /// \brief All the members in the team.
      public: std::vector<std::pair<int, std::string> > members;
    };

    /// \brief All the teams.
    public: std::vector<Team *> teams;
  };
}
#endif
