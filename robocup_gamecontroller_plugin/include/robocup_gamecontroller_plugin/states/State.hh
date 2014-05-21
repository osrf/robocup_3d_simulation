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

#ifndef _GAZEBO_STATE_PLUGIN_HH_
#define _GAZEBO_STATE_PLUGIN_HH_

#include <string>
#include <gazebo/gazebo.hh>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"

namespace gazebo
{
  class GameControllerPlugin;

  // \brief State pattern used for the game mode.
  class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[out] _plugin Reference to the GameControllerPlugin.
    public: State(const std::string &_name, GameControllerPlugin *_plugin);

    /// \brief Initialize the state. Called once when the state is entered.
    public: virtual void Initialize();

    /// \brief Update the state.
    public: virtual void Update() = 0;

    /// \brief Get the name of the state.
    /// \brief Returns the name of the state.
    public: std::string GetName();

    /// \brief Pointer to be able to access to the plugin inside the state.
    protected: GameControllerPlugin *plugin;

    /// \brief Name of the state.
    protected: std::string name;

    /// \brief Timer to measure the time elapsed in this state.
    protected: common::Timer timer;
  };
}
#endif
