#!/bin/bash

SESSION="ros2_uav"

tmux new-session -d -s $SESSION

tmux rename-window -t $SESSION 'xrce'
tmux send-keys "MicroXRCEAgent serial --dev /dev/pix4 -b 921600 & MicroXRCEAgent udp4 -p 8888" C-m

sleep 5

tmux new-window -t $SESSION -n 'camera'
tmux send-keys "ros2 launch usb_cam camera.launch.py" C-m

tmux new-window -t $SESSION -n 'vins'
tmux send-keys "ros2 launch vins_estimator vins_estimate.launch.py" C-m

tmux new-window -t $SESSION -n 'inference'
tmux send-keys "ros2 launch ros2_vision_inference metric_3d.launch.xml" C-m

sleep 5

tmux new-window -t $SESSION -n 'px4_com'
tmux send-keys "ros2 run px4_ros_com offboard_control" C-m

sleep 5

tmux new-window -t $SESSION -n 'planner'
tmux send-keys "ros2 launch ego_planner real_uav.launch.py" C-m

tmux attach-session -t $SESSION