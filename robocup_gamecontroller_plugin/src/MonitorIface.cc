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

#include <gazebo/physics/physics.hh>
#include "robocup_gamecontroller_plugin/MonitorIface.hh"

using namespace gazebo;

/////////////////////////////////////////////////
MonitorIface::MonitorIface(const std::string &_localAddress, unsigned int _port)
  : localAddress(_localAddress),
    port(_port),
    socket(new TCPServerSocket(_localAddress, _port))
{
  TCPSocket clientConnection = this->socket->accept();
  std::cout << "New connection established" << std::endl;
  this->SendDataTest(clientConnection);
}

/////////////////////////////////////////////////
MonitorIface::~MonitorIface()
{
}

/////////////////////////////////////////////////
void MonitorIface::SendDataTest(TCPSocket &_socket)
{
  std::string environment;
  environment = "((FieldLength 18)(FieldWidth 12)(FieldHeight 40)"
    "(GoalWidth 2.1)(GoalDepth 0.6)(GoalHeight 0.8)(FreeKickDistance 1.3)"
    "(WaitBeforeKickOff 2)(AgentRadius 0.4)(BallRadius 0.042)(BallMass 0.026)"
    "(RuleGoalPauseTime 3)(RuleKickInPauseTime 1)(RuleHalfTime 300)"
    "(play_modes BeforeKickOff KickOff_Left KickOff_Right PlayOn KickIn_Left "
    "KickIn_Right corner_kick_left corner_kick_right goal_kick_left "
    "goal_kick_right offside_left offside_right GameOver Goal_Left Goal_Right "
    "free_kick_left free_kick_right))";

  std::string header;
  header = "(RSG 0 1)";

  std::string body;
  body = "((nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 -10 10 10 1)(nd Light (setDiffuse 1 1 1 1) (setAmbient 0.8 0.8 0.8 1) (setSpecular 0.1 0.1 0.1 1)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 10 -10 10 1)(nd Light (setDiffuse 1 1 1 1) (setAmbient 0 0 0 1) (setSpecular 0.1 0.1 0.1 1)))"
  "(nd TRF (SLT -1 -8.74228e-008 -3.82137e-015 0 0 -4.37114e-008 1 0 -8.74228e-008 1 4.37114e-008 -0 0 0 0 1)(nd StaticMesh (load models/naosoccerfield.obj ) (sSc 1.5 1 1.5)(resetMaterials None_rcs-naofield.png)))"
  "(nd TRF (SLT -1 -8.74228e-008 -3.82137e-015 0 0 -4.37114e-008 1 0 -8.74228e-008 1 4.37114e-008 -0 0 0 0 1)(nd StaticMesh (load models/skybox.obj ) (sSc 10 10 10)(resetMaterials Material_skyrender_0001.tif Material_skyrender_0002.tif Material_skyrender_0003.tif Material_skyrender_0004.tif Material_skyrender_0005.tif Material_skyrender_0006.tif)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 -9.3 0 0.4 1)(nd TRF (SLT -4.37114e-008 1 4.37114e-008 0 0 -4.37114e-008 1 0 1 4.37114e-008 1.91069e-015 0 0.3 0 -0.4 1)(nd StaticMesh (setTransparent) (load models/leftgoal.obj) (sSc 2.18 0.88 0.68)(resetMaterials grey_naogoalnet.png yellow)))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 -1.07 0 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 1.07 0 1))(nd TRF (SLT 0.866025 0 -0.5 0 0 1 0 0 0.5 0 0.866025 0 -0.06 0 0 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0.3 1.05 0.4 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0.3 -1.05 0.4 1)))"
  "(nd TRF (SLT -1 -8.74228e-008 -0 -0 8.74228e-008 -1 0 0 0 0 1 0 9.3 0 0.4 1)(nd TRF (SLT -4.37114e-008 1 4.37114e-008 0 0 -4.37114e-008 1 0 1 4.37114e-008 1.91069e-015 0 0.3 0 -0.4 1)(nd StaticMesh (setTransparent) (load models/rightgoal.obj) (sSc 2.18 0.88 0.68)(resetMaterials grey_naogoalnet.png sky-blue white)))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 -1.07 0 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 1.07 0 1))(nd TRF (SLT 0.866025 0 -0.5 0 0 1 0 0 0.5 0 0.866025 0 -0.06 0 0 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0.3 1.05 0.4 1))(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0.3 -1.05 0.4 1)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 -14.994 0 0 1)(nd SMN (load StdUnitBox) (sSc 1 31 1) (sMat matGrey)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 14.994 0 0 1)(nd SMN (load StdUnitBox) (sSc 1 31 1) (sMat matGrey)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 15 0 1)(nd SMN (load StdUnitBox) (sSc 30.988 1 1) (sMat matGrey)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 -15 0 1)(nd SMN (load StdUnitBox) (sSc 30.988 1 1) (sMat matGrey)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 -9 6 0 1))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 -9 -6 0 1))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 9 6 0 1))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 9 -6 0 1))"
  "(nd TRF (SLT -0.319994 0.0257997 -0.947068 0 -0.763061 -0.599517 0.24149 0 -0.561553 0.799946 0.211529 0 3.01004 4.98996 0.0402713 1)(nd StaticMesh (load models/soccerball.obj ) (sSc 0.042 0.042 0.042)(resetMaterials soccerball_rcs-soccerball.png)))"
  "(nd TRF (SLT 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1)"
  " (nd TRF (SLT -3.44826e-005 -1 7.91397e-006 0 1 -3.44827e-005 -1.0609e-005 0 1.06092e-005 7.9136e-006 1 0 -8.2 5.2 0.383854 1)(nd TRF (SLT -1 3.82137e-015 8.74228e-008 -0 8.74228e-008 4.37114e-008 1 0 0 1 -4.37114e-008 0 0 0 0 1)(nd StaticMesh (load models/naobody.obj ) (sSc 0.1 0.1 0.1)(resetMaterials matNum1 matLeft naoblack naowhite))))"
  " (nd TRF (SLT -3.44826e-005 -17.9133e-006 0 1 -3.44827e-005 -1.06083e-005 0 1.06085e-005 7.91293e-006 1 0 -8.2 5.2 0.47385 1)(nd SMN (load StdCCylinder 0.015 0.08) (sSc 1 1 1) (sMat matDarkGrey)))"
  " (nd TRF (SLT -3.44826e-005 -1 7.91322e-006 0 1 -3.44827e-005 -1.06083e-005 0 1.06085e-005 7.91285e-006 1 0 -8.2 5.2 0.538847 1)(nd TRF (SLT -1 3.82137e-015 8.74228e-008 -0 8.74228e-008 4.37114e-008 1 0 0 1 -4.37114e-008 0 0 0 0 1)(nd StaticMesh (load models/naohead.obj ) (sSc 0.1 0.1 0.1)(resetMaterials matLeft naoblack naogrey naowhite))))"
  " (nd TRF (SLT -3.44834e-005 -1 7.89938e-006 0 1 -3.44835e-005 -1.06102e-005 0 1.06105e-005 7.89902e-006 1 0 -8.2 5.102 0.45885 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.44835e-005 -1 7.88479e-006 0 1 -3.44836e-005 -1.09411e-005 0 1.09414e-005 7.88442e-006 1 0 -8.18 5.092 0.458847 1)(nd StaticMesh (load models/rupperarm.obj) (sSc 0.07 0.07 0.07)(resetMaterials matLeft naoblack naowhite)))"
  " (nd TRF (SLT -3.44837e-005 -1 7.88479e-006 0 1 -3.44838e-005 -1.1038e-005 0 1.10382e-005 7.88441e-006 1 0 -8.11 5.102 0.467843 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.44837e-005 -1 7.88479e-006 0 1 -3.44838e-005 -1.11349e-005 0 1.11351e-005 7.88441e-006 1 0 -8.06 5.10199 0.467841 1)(nd StaticMesh (load models/rlowerarm.obj) (sSc 0.05 0.05 0.05)(resetMaterials matLeft naowhite)))"
  " (nd TRF (SLT -3.44833e-005 -1 7.92845e-006 0 1 -3.44834e-005 -1.06102e-005 0 1.06105e-005 7.92809e-006 1 0 -8.2 5.298 0.458849 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.44834e-005 -1 7.94294e-006 0 1 -3.44835e-005 -1.09407e-005 0 1.0941e-005 7.94256e-006 1 0 -8.18 5.308 0.458845 1)(nd StaticMesh (load models/lupperarm.obj) (sSc 0.07 0.07 0.07)(resetMaterials matLeft naoblack naowhite)))"
  " (nd TRF (SLT -3.44836e-005 -1 7.94293e-006 0 1 -3.44837e-005 -1.10374e-005 0 1.10377e-005 7.94255e-006 1 0 -8.11 5.298 0.467842 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.44837e-005 -1 7.94294e-006 0 1 -3.44838e-005 -1.11342e-005 0 1.11345e-005 7.94255e-006 1 0 -8.06 5.29799 0.467839 1)(nd StaticMesh (load models/llowerarm.obj) (sSc 0.05 0.05 0.05)(resetMaterials matLeft naowhite)))"
  " (nd TRF (SLT -3.42242e-005 -1 7.88087e-006 0 1 -3.42243e-005 -1.03488e-005 0 1.03491e-005 7.88052e-006 1 0 -8.21 5.145 0.268867 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.4224e-005 -1 7.88063e-006 0 1 -3.42241e-005 -9.83037e-006 0 9.83064e-006 7.88029e-006 1 0 -8.21 5.145 0.26888 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.42239e-005 -1 7.84752e-006 0 1 -3.42239e-005 -9.82846e-006 09.82873e-006 7.84719e-006 1 0 -8.2 5.145 0.228894 1)(nd StaticMesh (load models/rthigh.obj) (sSc 0.07 0.07 0.07)(resetMaterials matNum1 matLeft naowhite)))"
  " (nd TRF (SLT -3.42239e-005 -1 7.81824e-006 0 1 -3.42239e-005 -9.82679e-006 0 9.82706e-006 7.8179e-006 1 0 -8.19501 5.145 0.103911 1)(nd StaticMesh (load models/rshank.obj) (sSc 0.08 0.08 0.08)(resetMaterials matLeft naoblack naowhite)))"
  " (nd TRF (SLT -3.42239e-005 -1 7.79224e-006 0 1 -3.4224e-005 -9.82505e-006 0 9.82531e-006 7.7919e-006 1 0 -8.20501 5.145 0.0489304 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matRed)))"
  " (nd TRF (SLT -3.42239e-005 -1 7.792e-006 0 1 -3.42239e-005 -9.35416e-006 0 9.35442e-006 7.79168e-006 1 0 -8.17501 5.145 0.0139506 1)(nd StaticMesh (load models/rfoot.obj) (sSc 0.08 0.08 0.08)(resetMaterials matLeft naowhite)))"
  " (nd TRF (SLT -3.46705e-005 -1 7.95488e-006 0 1 -3.46705e-005 -1.04194e-005 0 1.04197e-005 7.95452e-006 1 0 -8.21 5.255 0.268866 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.46703e-005 -1 7.95494e-006 0 1 -3.46704e-005 -1.00419e-005 0 1.00421e-005 7.95459e-006 1 0 -8.21 5.255 0.268879 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matYellow)))"
  " (nd TRF (SLT -3.46701e-005 -1 7.99584e-006 0 1 -3.46702e-005 -1.00403e-005 0 1.00406e-005 7.99549e-006 1 0 -8.2 5.255 0.228893 1)(nd StaticMesh (load models/lthigh.obj) (sSc 0.07 0.07 0.07)(resetMaterials matLeft naowhite)))"
  " (nd TRF (SLT -3.46701e-005 -1 8.04058e-006 0 1 -3.46702e-005 -1.00389e-005 0 1.00392e-005 8.04024e-006 1 0 -8.195 5.255 0.10391 1)(nd StaticMesh (load models/lshank.obj) (sSc 0.08 0.08 0.08)(resetMaterials matLeft naoblack naowhite)))"
  " (nd TRF (SLT -3.46701e-005 -1 8.08863e-006 0 1 -3.46702e-005 -1.00375e-005 0 1.00378e-005 8.08828e-006 1 0 -8.205 5.255 0.0489293 1)(nd SMN (load StdUnitSphere) (sSc 0.01 0.01 0.01) (sMat matRed)))"
  " (nd TRF (SLT -3.46701e-005 -1 8.0887e-006 0 1 -3.46702e-005 -9.70015e-006 0 9.70043e-006 8.08836e-006 1 0 -8.175 5.255 0.0139494 1)(nd StaticMesh (load models/lfoot.obj) (sSc 0.08 0.08 0.08)(resetMaterials matLeft naowhite)))"
  ")"
  ")";

  std::string gameState;
  gameState = "((time 600.003)(team_left TinManBots)(half 2)(score_left 0)(score_right 0)(play_mode 12))"
 "(RDS 0 1)"
 "((nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd(nd))(nd)(nd)(nd)(nd)(nd))(nd(nd(nd))(nd)(nd)(nd)(nd)(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd)(nd)(nd)(nd)(nd(nd))(nd(nd(nd(nd)))(nd(nd))(nd(nd(nd)))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))))";

 std::string update;
 update = "((time 600.003))"
 "(RDS 0 1)"
 "((nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd(nd))(nd)(nd)(nd)(nd)(nd))(nd(nd(nd))(nd)(nd)(nd)(nd)(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd)(nd)(nd)(nd)(nd(nd))(nd(nd(nd(nd)))(nd(nd))(nd(nd(nd)))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))(nd(nd))))";
}
