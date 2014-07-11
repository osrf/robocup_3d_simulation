#!/bin/sh

# Spawn the left team
for i in $(seq 1 1)
do
	rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/nao_soccer.sdf teamA 0 &
done

# Spawn the right team
#for i in $(seq 1 1)
#do
#	rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/nao_soccer.sdf teamB 0 &
#done
