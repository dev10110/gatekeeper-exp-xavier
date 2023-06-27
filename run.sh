#!/bin/bash

COLCON_WS=$PWD/colcon_ws

cd ${COLCON_WS}/src/isaac_ros_common && \
	./scripts/run_dev.sh ${COLCON_WS}
