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

#ifndef _GAZEBO_SOCCER_FIELD_PLUGIN_HH_
#define _GAZEBO_SOCCER_FIELD_PLUGIN_HH_

#include <gazebo/math/Box.hh>
#include <gazebo/math/Pose.hh>

namespace gazebo
{
  namespace SoccerField
  {
    // Field dimensions.
    static const double FieldWidth = 20.0;
    static const double HalfFieldWidth = SoccerField::FieldWidth * 0.5;
    static const double FieldHeight = 30.0;
    static const double HalfFieldHeight = SoccerField::FieldHeight * 0.5;
    static const double FreeKickMoveDist = 2.0;
    static const double FreeKickDist = 9.15;
    static const double GoalWidth = 2.1;
    static const double HalfGoalWidth = SoccerField::GoalWidth * 0.5;

    static const math::Box FieldLeft(
      math::Vector3(
        -SoccerField::HalfFieldHeight, -SoccerField::HalfFieldWidth, 0),
      math::Vector3(0, SoccerField::HalfFieldWidth, 0));

    static const math::Box FieldRight(
      math::Vector3(0, -SoccerField::HalfFieldWidth, 0),
      math::Vector3(
        SoccerField::HalfFieldHeight, SoccerField::HalfFieldWidth, 0));

    // Initial player positions
    const math::Pose LeftInitPoseKickOff1(math::Pose(-0.2, -0.3, 0, 0, 0, 0.5));
    const math::Pose LeftInitPoseKickOff2(
      math::Pose(-0.2, 0.3, 0, 0, 0, -0.5));
    const math::Pose LeftInitPoseKickOff3(math::Pose(-2.0, -0.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff4(math::Pose(-5.0, 2.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff5(
      math::Pose(-5.0, -2.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff6(math::Pose(-5.0, 0.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff7(math::Pose(-10.0, 3.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff8(
      math::Pose(-10, 1.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff9(math::Pose(-10.0, -1.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff10(math::Pose(-10.0, -3.5, 0, 0, 0, 0));
    const math::Pose LeftInitPoseKickOff11(
      math::Pose(-SoccerField::HalfFieldHeight + 0.5, 0, 0, 0, 0, 0));

    const math::Pose LeftInitPose1(math::Pose(-2.5, 0, 0, 0, 0, 0));
    const math::Pose LeftInitPose2(math::Pose(-3.5, -2.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose3(math::Pose(-3.5, 0, 0, 0, 0, 0));
    const math::Pose LeftInitPose4(math::Pose(-3.5, 2.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose5(math::Pose(-5.5, -4.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose6(math::Pose(-5.5, 0, 0, 0, 0, 0));
    const math::Pose LeftInitPose7(math::Pose(-5.5, 4.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose8(math::Pose(-7.5, -5.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose9(math::Pose(-7.5, 0, 0, 0, 0, 0));
    const math::Pose LeftInitPose10(math::Pose(-7.5, 5.0, 0, 0, 0, 0));
    const math::Pose LeftInitPose11(
      math::Pose(-SoccerField::HalfFieldHeight + 0.5, 0, 0, 0, 0, 0));

    const math::Pose RightInitPoseKickOff1(math::Pose(0.5, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff2(
      math::Pose(2.0, -3.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff3(math::Pose(2.0, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff4(math::Pose(2.0, 3.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff5(
      math::Pose(4.0, -4.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff6(math::Pose(4.0, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff7(math::Pose(4.0, 4.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff8(
      math::Pose(6.0, -5.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff9(math::Pose(6.0, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff10(math::Pose(6.0, 5.5, 0, 0, 0, 3.14));
    const math::Pose RightInitPoseKickOff11(
      math::Pose(SoccerField::HalfFieldHeight - 0.5, 0, 0, 0, 0, 3.14));

    const math::Pose RightInitPose1(math::Pose(2.5, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose2(math::Pose(3.5, -2.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose3(math::Pose(3.5, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose4(math::Pose(3.5, 2.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose5(math::Pose(5.5, -4.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose6(math::Pose(5.5, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose7(math::Pose(5.5, 4.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose8(math::Pose(7.5, -5.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose9(math::Pose(7.5, 0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose10(math::Pose(7.5, 5.0, 0, 0, 0, 3.14));
    const math::Pose RightInitPose11(
      math::Pose(SoccerField::HalfFieldHeight - 0.5, 0, 0, 0, 0, 3.14));
  }
}
#endif
