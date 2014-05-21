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

#ifndef _GAZEBO_CORNER_KICK_LEFT_STATE_HH_
#define _GAZEBO_CORNER_KICK_LEFT_STATE_HH_

#include <string>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/State.hh"

namespace gazebo
{
  /// \class CornerKickLeftState CornerKickLeftState.hh
  /// \brief State that handles the corner kick left state.
  class CornerKickLeftState : public State
  {
    /// \brief Constructor.
    /// \param[in] _name Name of this state.
    /// \param[in] _plugin GameControllerPlugin to be used inside the state.
    public: CornerKickLeftState(const std::string &_name,
                                GameControllerPlugin *_plugin);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };
}
#endif
