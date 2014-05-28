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
#include <gazebo/physics/physics.hh>
#include "robocup_agent_plugin/AgentPlugin.hh"
#include "robocup_msgs/SendJoints.h"
// #include "robocup_msgs/Say.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AgentPlugin)

/////////////////////////////////////////////////
AgentPlugin::AgentPlugin()
{
	// Initialize the joint names.
	this->jointNames.push_back("HeadYaw");
	this->jointNames.push_back("HeadPitch");
	this->jointNames.push_back("LHipYawPitch");
	this->jointNames.push_back("LHipRoll");
	this->jointNames.push_back("LHipPitch");
	this->jointNames.push_back("LKneePitch");
	this->jointNames.push_back("LAnklePitch");
	this->jointNames.push_back("LAnkleRoll");
	this->jointNames.push_back("LShoulderPitch");
	this->jointNames.push_back("LShoulderRoll");
	this->jointNames.push_back("LElbowYaw");
	this->jointNames.push_back("LWristYaw");
	this->jointNames.push_back("RHipYawPitch");
	this->jointNames.push_back("RHipRoll");
	this->jointNames.push_back("RHipPitch");
	this->jointNames.push_back("RKneePitch");
	this->jointNames.push_back("RAnklePitch");
	this->jointNames.push_back("RAnkleRoll");
	this->jointNames.push_back("RShoulderPitch");
	this->jointNames.push_back("RShoulderRoll");
	this->jointNames.push_back("RElbowYaw");
	this->jointNames.push_back("RWristYaw");

  // Start up ROS
  std::string name = "set_joints";
  int argc = 0;
  ros::init(argc, NULL, name);

  gzlog << "RoboCup 3D simulator agent plugin running" << std::endl;

  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  // ROS Nodehandle.
  this->node.reset(new ros::NodeHandle("~"));
}

/////////////////////////////////////////////////
AgentPlugin::~AgentPlugin()
{
}

/////////////////////////////////////////////////
void AgentPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	this->modelName = _sdf->Get<std::string>("robot_namespace");

	this->model = _model;

	// Advertise the service for sending new joint commands.
  this->jointCommandsService =
    this->node->advertiseService("/" + this->modelName + "/send_joints",
    &AgentPlugin::SendJoints, this);

  /* Info: The agent has to advertise a topic '/<robot_name>/say' and publish
   a message in this topic each the agent wants to "say" something. The
   game controller plugin is subscribed to this topic and will forward the
   message to the rest of the teammates.

   In a similar way, each agent has to subscribe to a topic
   '/<robot_name>/listen', where the gamecontroller will forward the messages
   coming from other robots.*/

   // Debugging.
   // Register the callback for the 'listen' message.
  /* this->listenSub = this->node->subscribe<robocup_msgs::Say>(
    std::string("/" + this->modelName + "/listen"), 1000,
    &AgentPlugin::OnMessageFromRobot, this);*/

  // Debugging
  // Advertise the topic for sending messages to other robots.
  // this->sayPub = this->node->advertise<robocup_msgs::Say>(
  //  std::string("/" + this->modelName + "/say"), 10);
  // robocup_msgs::Say msg;
  //	this->sayPub.publish(msg);
}

/////////////////////////////////////////////////
void AgentPlugin::Init()
{
}

/////////////////////////////////////////////////
bool AgentPlugin::SendJoints(
  robocup_msgs::SendJoints::Request  &req,
  robocup_msgs::SendJoints::Response &res)
{
	boost::array<float, 22> jointValues = req.joints;

	for (int i = 0; i < 22; ++i)
	{
		// Get the joint.
		physics::JointPtr joint =
		  this->model->GetJoint(this->modelName + "::Nao::" + this->jointNames[i]);
    if (!joint)
    {
      std::cerr << "SendJoints() Joint [" << this->modelName << "::Nao::"
                << this->jointNames[i] << "] not found" << std::endl;
      continue;
    }

		// Set the force for this joint.
    joint->SetForce(0, jointValues[i]);
	}
}

/////////////////////////////////////////////////
// Debugging
/*void AgentPlugin::OnMessageFromRobot(const robocup_msgs::Say::ConstPtr& _msg)
{
	boost::array<signed char, 20> message = _msg->message;
	std::cout << this->modelName << ": New message received from other robot."
	          << std::endl;
}*/
