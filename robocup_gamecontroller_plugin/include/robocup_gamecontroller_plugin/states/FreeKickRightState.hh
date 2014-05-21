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

#ifndef _GAZEBO_FREE_KICK_RIGHT_STATE_HH_
#define _GAZEBO_FREE_KICK_RIGTH_STATE_HH_

#include <string>
#include <gazebo/math/Vector3.hh>
#include "robocup_gamecontroller_plugin/GameControllerPlugin.hh"
#include "robocup_gamecontroller_plugin/states/State.hh"

namespace gazebo
{
  /// \class FreeKickRightState FreeKickRightState.hh
  /// \brief State that handles the free kick right state.
  class FreeKickRightState : public State
  {
    /// Documentation inherited.
    public: FreeKickRightState(const std::string &_name,
                               GameControllerPlugin *_plugin);

    /// Documentation inherited.
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();

    /// \brief Set the position where the ball will be located.
    /// \param[in] _ballPos Desired ball position.
    public: void SetPos(const math::Vector3 &_pos);

    /// \brief Used to select the position where the ball will be moved.
    private: math::Vector3 pos;
  };
}
#endif
