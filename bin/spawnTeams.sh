#!/bin/sh

# Spawn the left team
for i in $(seq 1 2)
do
	rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/nao_soccer_blue.sdf teamA 0 &
done

sleep 1

# Spawn the right team
for i in $(seq 1 0)
do
	rosservice call /gameController/init_agent ~/workspace/robocup_3d_simulation/models/nao_soccer_red.sdf teamB 0 &
done
