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

#ifndef _GAZEBO_MONITOR_INTERFACE_HH_
#define _GAZEBO_MONITOR_INTERFACE_PLUGIN_HH_

#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include "robocup_gamecontroller_plugin/Socket.hh"

namespace gazebo
{
  class MonitorIface
  {
    /// \brief Constructor.
    public: MonitorIface(const std::string &_localAddress,
      unsigned int _port = 3200);

    /// \brief Destructor.
    public: virtual ~MonitorIface();

    public: void SendDataTest(TCPSocket &_socket);

    /// \brief IP address to be bind.
    private: std::string localAddress;

    /// \brief TCP port to be bind.
    private: unsigned int port;

    private: boost::scoped_ptr<TCPServerSocket> socket;
  };
}
#endif
