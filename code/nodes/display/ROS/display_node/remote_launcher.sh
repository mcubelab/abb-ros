#!/bin/bash
export DISPLAY=:0; rosrun display_node display_node
exec $@
