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

#include <gazebo/gazebo.hh>
#include <boost/thread/mutex.hpp>
#include <string>
#include "robocup_msgs/InitAgent.h"
#include "robocup_msgs/SetGameState.h"

namespace gazebo
{
  class State;

  class GameControllerPlugin : public WorldPlugin
  {
    public: static const std::string Kickoff;
    public: static const std::string Playing;
    public: static const std::string Finished;
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
    private: void SetCurrent(State *_newState);

    /// \brief ROS service callback that sets the state of the game.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call response.
    /// \return True when the service call succeeds.
    private: bool SetGameState(robocup_msgs::SetGameState::Request  &req,
                               robocup_msgs::SetGameState::Response &res);

    /// \brief Executes the update on the current state.
    private: void Update();

    /// \brief Update the robocup simulation state.
    /// \param[in] _info Information used in the update event.
    private: void UpdateStates(const common::UpdateInfo &_info);

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Pointer to the soccer ball.
    public: physics::ModelPtr ball;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    // ROS Node handler
    private: ros::NodeHandle* node;

    // ROS Subscriber
    private: ros::Subscriber sub;

    // ROS Service for spawning new agents.
    private: ros::ServiceServer initAgentService;

    // ROS Service for change the state of the game.
    private: ros::ServiceServer setGameStateService;

    // ROS Publisher.
    private: ros::Publisher publisher;

    /// \brief Game half (1st half or 2nd half).
    private: uint8_t half;

    /// \brief Left team score.
    private: uint32_t scoreLeft;

    /// \brief Right team score.
    private: uint32_t scoreRight;

    /// \brief Sim time at which the game started.
    private: gazebo::common::Time startTimeSim;

    /// \brief Pointer to the current game state.
    private: State *currentState;

    /// \brief Pointer to the kickoff state.
    private: State *kickoffState;

    /// \brief Pointer to the play state.
    private: State *playState;

    /// \brief Pointer to the finished state.
    private: State *finishedState;

    /// \brief Game time.
    private: common::Time elapsedTimeSim;

    /// \brief Mutex to avoid race conditions while running updates and a ROS
    /// callback is executed.
    private: boost::mutex mutex;

   /* private: void ClearPlayers(const math::Box &_box, double _minDist,
                 unsigned int _teamIndex); */

    /// \brief A single team.
    /*private: class Team
    {
      /// \brief Name of the team.
      public: std::string name;

      /// \brief All the members in the team.
      public: std::vector<physics::ModelPtr> members;
    };*/

    /// \brief All the teams.
    //private: std::vector<Team *> teams;

    /// \brief Current state
    //private: State *state;

  };

  // \brief State pattern used for the game mode.
  class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[out] _plugin Reference to the GameControllerPlugin.
    public: State(const std::string &_name,
                  GameControllerPlugin *_plugin);

    /// \brief Initialize the state. Called once when the state is entered.
    public: virtual void Initialize() = 0;

    /// \brief Update the state.
    public: virtual void Update() = 0;

    /// \brief Get the name of the state.
    /// \brief Returns the name of the state.
    public: std::string GetName();

    /// \brief Pointer to be able to access to the plugin inside the state.
    protected: GameControllerPlugin *plugin;

    /// \brief Name of the state.
    protected: std::string name;
  };

  /// \brief State that handles finished.
  class FinishedState : public State
  {
    /// Documentation inherited.
    public: FinishedState(const std::string &_name,
                          GameControllerPlugin *_plugin);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };

  /// \brief State that handles kickoff.
  class KickoffState : public State
  {
    /// Documentation inherited.
    public: KickoffState(const std::string &_name,
                         GameControllerPlugin *_plugin);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };

  /// \brief State that handels normal play.
  class PlayState : public State
  {
    /// Documentation inherited.
    public: PlayState(const std::string &_name,
                      GameControllerPlugin *_plugin);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };
}
#endif
