#!/bin/bash

args=$*
shift $#

echo "args: $args"

cd /workspaces
colcon build --packages-select revo2_description > /dev/null
source install/setup.bash

# Default to left hand if no argument provided
hand=${args:-left}

echo "hand: $hand"

if [[ "$hand" == "left" ]]; then
    ros2 launch revo2_description view_revo2_left_hand.launch.py
else
    ros2 launch revo2_description view_revo2_right_hand.launch.py
fi
