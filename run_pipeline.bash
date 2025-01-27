tmux new-session -d;  # start new detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && cd ~/Documents/rosbags && ros2 bag record -a' ENTER;                    
tmux split-window -h -p 50;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 ' ENTER;                    
tmux split-window -v -p 50;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 run lidar lidar_sub' ENTER;                    
tmux split-window -h -p 100;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 launch zed_wrapper zed2.launch.py' ENTER;                    
tmux split-window -v -p 70;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 run stereo stereo_cones' ENTER;                    
tmux split-window -h -p 70;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 launch sbg_device sbg_device_launch.py' ENTER;                    
tmux split-window -v -p 70;                             # split the detached tmux session
tmux send 'source ~/Documents/movella_ws/install/setup.bash && ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py' ENTER;                    
tmux split-window -h -p 70;                             # split the detached tmux session
tmux a;                                                 # open (attach) tmux session.