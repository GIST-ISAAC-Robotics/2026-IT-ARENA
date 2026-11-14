# Source from a clean Bash shell after selecting the physical ROS domain.
: "${ROS_DOMAIN_ID:?Select a physical ROS domain distinct from simulation first}"
source /opt/ros/humble/setup.bash
source /home/q/it-arena-field-r500-20261114-a/install/local_setup.bash
export PYTHONPATH="/home/q/it-arena-field-r500-20261114-a/deps:/home/q/it-arena-field-r500-20261114-a/scripts:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# No runtime profile, device, policy, node or firmware is selected or started here.
