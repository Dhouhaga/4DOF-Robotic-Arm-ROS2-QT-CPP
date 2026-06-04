#!/bin/bash
PI="pi4@pi4.local"

rsync -av --delete ./src/ $PI:~/ros2_ws/src/

ssh $PI "source /opt/ros/jazzy/setup.sh && \
         cd ~/ros2_ws && \
         colcon build && \
         source install/setup.bash"


echo "src folder is on the pi now under ~/ros2_ws/src/ and the workspace is built"