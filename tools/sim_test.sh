#!/bin/bash
# Run the PC stack (pc_bringup.launch.py) against the simulated car, on an
# isolated ROS domain that only this machine can see, so the real car is
# never touched. Ctrl-C stops everything.
#
#   tools/sim_test.sh                 # EKF + SLAM + RViz
#   tools/sim_test.sh nav2:=true motors_on:=true   # also Nav2: send goals with Nav2 Goal
#
# Any arguments are passed to pc_bringup.launch.py. The simulated car's true
# pose is printed once a second as "TRUTH x y yaw_deg".
set -e
cd "$(dirname "$0")/.."
REPO=$PWD

export ROS_DOMAIN_ID=${SIM_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
echo "Simulation on ROS domain $ROS_DOMAIN_ID, localhost only"

python3 "$REPO/tools/sim_car.py" &
SIM=$!
# The binary itself, not through ros2 run: killing the ros2 run wrapper left
# robot_state_publisher running, publishing a stale TF on this domain
"$(ros2 pkg prefix robot_state_publisher)/lib/robot_state_publisher/robot_state_publisher" --ros-args \
  -p robot_description:="$(cat "$REPO/urdf/robot_car.urdf.xml")" > /dev/null 2>&1 &
RSP=$!
trap 'kill $SIM $RSP 2>/dev/null' EXIT
sleep 2

ros2 launch car_s pc_bringup.launch.py "$@"
