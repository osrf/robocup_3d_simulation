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
#include <string>
#include <vector>
#include "std_msgs/String.h"


namespace gazebo
{
  // \brief Generic state of the game.
  /*class State
  {
    /// \brief Initialize the state. This is called once when the state is
    /// entered.
    public: virtual void Init() = 0;

    /// \brief Update the state.
    /// \return Pointer to the new state.
    public: virtual State *Update() = 0;

    protected: std::vector<State*> children;
  };

  /// \brief State that handles kickoff.
  class KickoffState : public State
  {
    /// \brief Move players and ball before kickoff.
    public: virtual void Init();

    // Documentation inherited
    public: virtual State *Update();
  };

  /// \brief State that handels normal play.
  class PlayState : public State
  {
    // Documentation inherited
    public: virtual State *Update();
  };*/

  class GameControllerPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: GameControllerPlugin();

    /// \brief Destructor.
    public: virtual ~GameControllerPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    public: void CreateEffector(const std_msgs::String::ConstPtr& msg);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the robocup simulation state.
    public: void UpdateStates(const common::UpdateInfo &_info);

    private: void ClearPlayers(const math::Box &_box, double _minDist,
                 unsigned int _teamIndex);

    /// \brief A single team.
    private: class Team
    {
      /// \brief Name of the team.
      public: std::string name;

      /// \brief All the members in the team.
      public: std::vector<physics::ModelPtr> members;
    };

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;

    /// \brief Pointer to the soccer ball.
    private: physics::ModelPtr ball;

    /// \brief All the teams.
    private: std::vector<Team *> teams;

    /// \brief Current state
    //private: State *state;

    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    // ROS Subscriber
    private: ros::Subscriber sub;
  };
}
#endif
