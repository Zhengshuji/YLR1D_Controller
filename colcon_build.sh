export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src
rm -r build install log
colcon build

# export AMENT_PREFIX_PATH="/home/zsj/WorkSpace/SWurdf/install/ylr1d_description:$AMENT_PREFIX_PATH"

#source install/setup.bash
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/zsj/WorkSpace/test_ylr1d/src



ps aux | grep gazebo
pkill -f gzserver
pkill -f gazebo