envCmd="/opt/ros/humble/setup.bash"
envCmdLocal="./install/local_setup.sh"
# source "/opt/ros/humble/setup.bash"
source $envCmd
source $envCmdLocal
ros2
ros2 run ssros_ts talker
