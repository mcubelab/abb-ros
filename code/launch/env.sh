#!/bin/sh



if [ $# -eq 0 ] ; then
    /bin/echo "Entering environment at /opt/ros/fuerte"
    . /opt/ros/fuerte/setup.sh
    export ROS_PACKAGE_PATH=/home/simplehands/Documents/hands/code:$ROS_PACKAGE_PATH
    $SHELL
    /bin/echo "Exiting build environment at /opt/ros/fuerte"
else
    . /opt/ros/fuerte/setup.sh
    export ROS_PACKAGE_PATH=/home/simplehands/Documents/hands/code:$ROS_PACKAGE_PATH
    exec "$@"
fi
