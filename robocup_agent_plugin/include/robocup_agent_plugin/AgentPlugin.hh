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
#include <boost/scoped_ptr.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include "robocup_msgs/GameStateMonitor.h"
#include "robocup_msgs/SendJoints.h"
#include <string>
#include <vector>

#include "robocup_msgs/AgentState.h"

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

    private: void GameStateCb(const robocup_msgs::GameStateMonitor &_msg);

    // Debugging
    /// \brief ROS message callback to receive messages from other robots.
    /// \param[in] _msg Message sent from other player.
    // void OnMessageFromRobot(const robocup_msgs::Say::ConstPtr& _msg);

    /// \brief Update the plugin. Called every simulation iteration.
    private: void Update(const common::UpdateInfo &_info);

    private: void SendState();
    private: void SendLines(robocup_msgs::AgentState &_msg);

    /// \brief Connection to the update event.
    private: event::ConnectionPtr updateConnection;

    /// \brief Joints that comprise the robot's state.
    private: std::vector<physics::JointPtr> joints;

    /// \brief ROS node
    private: boost::scoped_ptr<ros::NodeHandle> node;

    private: ros::Subscriber subscriber;

    /// \brief Publisher of agent state information.
    private: ros::Publisher agentStatePub;

    /// \brief Controls when the state message is sent.
    private: unsigned int stateMsgCounter;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the ball model
    private: physics::ModelPtr ball;

    /// \brief Camera position after applying some noise.
    private: math::Pose cameraPoseNoise;

    /// \brief Noise parameters.
    private: double distanceNoiseSigma;
    private: double angle1NoiseSigma;
    private: double angle2NoiseSigma;

    /// \brief Nao sensors.
    private: std::vector<sensors::ImuSensorPtr> imuSensors;
    private: std::vector<sensors::ContactSensorPtr> touchSensors;
    private: std::vector<sensors::ContactSensorPtr> forceSensors;

    /// \brief Perception view angle.
    private: double viewAngle;

    private: robocup_msgs::GameStateMonitor gameState;

    private: class Line
    {
      public: Line(const math::Vector2d &_pt1, const math::Vector2d &_pt2)
              : pt1(_pt1), pt2(_pt2) {}

      public: Line(double _x1, double _y1, double _x2, double _y2)
              : pt1(_x1, _y1), pt2(_x2, _y2) {}

      public: bool Intersect(const Line &_line, math::Vector2d &_result) const
      {
        double a = this->pt1.x - this->pt2.x;
        double b = _line.pt1.y-_line.pt2.y;
        double c = this->pt1.y - this->pt2.y;
        double e = _line.pt1.x-_line.pt2.x;

        double d = (this->pt1.x - this->pt2.x) * (_line.pt1.y-_line.pt2.y) -
                   (this->pt1.y - this->pt2.y) * (_line.pt1.x-_line.pt2.x);

        if (math::equal(d, 0.0))
          return false;

        _result.x = (_line.pt1.x - _line.pt2.x) *
                   (this->pt1.x * this->pt2.y - this->pt1.y * this->pt2.x) -
                   (this->pt1.x - this->pt2.x) *
                   (_line.pt1.x * _line.pt2.y - _line.pt1.y * _line.pt2.x);

        _result.y = (_line.pt1.y - _line.pt2.y) *
                   (this->pt1.x * this->pt2.y - this->pt1.y * this->pt2.x) -
                   (this->pt1.y - this->pt2.y) *
                   (_line.pt1.x * _line.pt2.y - _line.pt1.y * _line.pt2.x);
        _result /= d;

        if (_result.x < std::min(this->pt1.x,this->pt2.x) ||
            _result.x > std::max(this->pt1.x,this->pt2.x) ||
            _result.x < std::min(_line.pt1.x,_line.pt2.x) ||
            _result.x > std::max(_line.pt1.x,_line.pt2.x))
        {
          return false;
        }

        if (_result.y < std::min(this->pt1.y, this->pt2.y) ||
            _result.y > std::max(this->pt1.y, this->pt2.y) ||
            _result.y < std::min(_line.pt1.y, _line.pt2.y) ||
            _result.y > std::max(_line.pt1.y, _line.pt2.y))
        {
          return false;
        }

        return true;
      }

      public: math::Vector2d pt1;
      public: math::Vector2d pt2;
    };

    private: class Triangle
    {
      public: Triangle(const math::Vector2d &_pt1,
                  const math::Vector2d _pt2,
                  const math::Vector2d &_pt3)
              : pt1(_pt1), pt2(_pt2), pt3(_pt3) {}

      public: bool Contains(const Line &_line) const
              {
                return this->Contains(_line.pt1) && this->Contains(_line.pt2);
              }

      public: bool Contains(const math::Vector2d &_pt) const
              {
                // Compute vectors
                math::Vector2d v0 = this->pt3 -this->pt1;
                math::Vector2d v1 = this->pt2 -this->pt1;
                math::Vector2d v2 = _pt - this->pt1;

                // Compute dot products
                double dot00 = v0.Dot(v0);
                double dot01 = v0.Dot(v1);
                double dot02 = v0.Dot(v2);
                double dot11 = v1.Dot(v1);
                double dot12 = v1.Dot(v2);

                // Compute barycentric coordinates
                double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
                double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
                double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

                // Check if point is in triangle
                return (u >= 0) && (v >= 0) && (u + v < 1);
              }

      public: bool Intersects(const Line &_line,
                              math::Vector2d &_ipt1,
                              math::Vector2d &_ipt2) const
              {
                if (this->Contains(_line))
                {
                  _ipt1 = _line.pt1;
                  _ipt2 = _line.pt2;
                  return true;
                }

                Line line1(this->pt1, this->pt2);
                Line line2(this->pt2, this->pt3);
                Line line3(this->pt3, this->pt1);

                math::Vector2d pt;
                std::vector<math::Vector2d> pts;
                if (line1.Intersect(_line, pt))
                {
                  pts.push_back(pt);
                }

                if (line2.Intersect(_line, pt))
                {
                  pts.push_back(pt);
                }

                if (line3.Intersect(_line, pt))
                {
                  pts.push_back(pt);
                }

                if (pts.empty())
                  return false;
                else if (pts.size() == 1)
                {
                  _ipt1 = pts[0];
                  if (this->Contains(_line.pt1))
                    _ipt2 = _line.pt1;
                  else
                    _ipt2 = _line.pt2;
                }
                else
                {
                  _ipt1 = pts[0];
                  _ipt2 = pts[1];
                }

                return true;
              }

      public: math::Vector2d pt1;
      public: math::Vector2d pt2;
      public: math::Vector2d pt3;
    };

    /// \brief Mutex to avoid race conditions while running updates and a ROS
    /// callback is executed.
    private: boost::mutex mutex;

    /// \brief Pointer to a node for communication.
    private: transport::NodePtr gzNode;

    // Debugging.
    // ROS subscription for receiving other player messages.
    // private: ros::Subscriber listenSub;

    // Debugging.
    // ROS publisher for sending messages to other players.
    // private: ros::Publisher sayPub;

    // ROS Service for spawning new agents.
    private: ros::ServiceServer jointCommandsService;

    /// \brief Model name.
    private: std::string modelName;

    /// \brief Vector with all the joint names.
    private: std::vector<std::string> jointNames;

    /// \brief List of lines seen from the camera point of view.
    private: std::vector<Line> lines;

    /// \brief Name of the link considered head. This link is a parameter to be
    /// able to use different robots during the same game.
    private: gazebo::physics::LinkPtr headLink;

    /// \brief Helper map for converting joint names between the agents and the
    /// the server.
    private: std::map<std::string, std::string> toAgent;
    private: std::map<std::string, std::string> toServer;
  };
}
#endif
