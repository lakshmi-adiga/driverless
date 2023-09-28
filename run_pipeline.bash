tmux new-session -d;  # start new detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 launch hesai_ros_driver start.py' ENTER;                    
tmux split-window -h -p 100;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 run perceptions lidar_sub' ENTER;                    
tmux split-window -v -p 120;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && cd ~/Documents/driverless/driverless_ws && ros2 run stereo stereo_cones' ENTER;                    
tmux split-window -h -p 100;                             # split the detached tmux session
tmux send 'source ~/Documents/driverless/driverless_ws/install/setup.bash && ros2 launch sbg_driver sbg_device_launch.py' ENTER; 
tmux split-window -v -p 100;                             # split the detached tmux session
tmux send 'source ~/movella_ws/install/setup.bash && ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py' ENTER;                    
tmux a;                                                 # open (attach) tmux session.