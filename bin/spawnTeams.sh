#!/bin/sh

# Spawn the left team
rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/teamA_1.sdf team1 1

#rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/teamA_2.sdf team1 2

# Spawn the right team
#rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/teamB_1.sdf team2 1

#rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/teamB_2.sdf team2 2