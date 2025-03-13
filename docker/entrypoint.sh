#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /colcon_ws/install/setup.bash ]
then
  source /colcon_ws/install/setup.bash
fi

exec "$@"