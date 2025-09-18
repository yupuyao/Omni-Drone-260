#!/bin/bash

SESSION="ros2_uav"

tmux new-session -d -s $SESSION

sleep 20

tmux rename-window -t $SESSION 'xrce'
tmux send-keys "MicroXRCEAgent serial --dev /dev/pix4 -b 921600" C-m

sleep 10


tmux new-window -t $SESSION -n 'offboard'
tmux send-keys "ros2 run px4_ros_com offboard_control" C-m


sleep 5

tmux new-window -t $SESSION -n 'planner'
tmux send-keys "ros2 run trajectory_input trajectory_input_node" C-m


tmux attach-session -t $SESSION
