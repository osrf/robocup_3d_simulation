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
#include <gazebo/physics/physics.hh>
#include "robocup_msgs/SendJoints.h"
#include <string>
#include <vector>

namespace gazebo
{
  class AgentPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: AgentPlugin();

    /// \brief Destructor.
    public: virtual ~AgentPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief ROS service callback to send joint commands to the simulation.
    /// \param[out] _req ROS service call request.
    /// \param[out] _res ROS service call result.
    /// \return True when the service call succeeds.
    private: bool SendJoints(robocup_msgs::SendJoints::Request  &_req,
                             robocup_msgs::SendJoints::Response &_res);

    // ROS Node handler
    private: boost::scoped_ptr<ros::NodeHandle> node;

    /// \brief Mutex to avoid race conditions while running updates and a ROS
    /// callback is executed.
    private: boost::mutex mutex;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    // ROS Service for spawning new agents.
    private: ros::ServiceServer jointCommandsService;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Model name.
    private: std::string modelName;

    /// \brief Vector with all the joint names.
    private: std::vector<std::string> jointNames;
  };
}
#endif
