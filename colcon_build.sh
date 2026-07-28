export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src
conda activate ros2
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src
source install/setup.bash

rm -r build install log
colcon build --packages-select ylr1d_mid_control --cmake-clean-cache

# export AMENT_PREFIX_PATH="/home/zsj/WorkSpace/SWurdf/install/ylr1d_description:$AMENT_PREFIX_PATH"

#source install/setup.bash
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src

#ps aux | grep gazebo
#pkill -f gzserver
#pkill -f gazebo

# ros2 launch ylr1d_mid_control gazebo.launch.py 2>&1 | tee launch.log