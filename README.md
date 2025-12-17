SETUP:
1. run rosdep install --from-paths src --ignore-src -r -y

2. launch: ros2 launch leg_assembly display.launch.py

3. move group node launch :
     on separate terminal, first run "ros2 run joy joy_node"


    then, after sourcing workspace, run "ros2 run teleop_collision teleop_collision"


:D

