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
# Processes killed hard (kill -9, a crash) leave Fast DDS shared-memory
# files in /dev/shm, and enough of them broke discovery between later runs
# (explore_lite never found Nav2). Clear the ones nothing is using
command -v fastdds > /dev/null && fastdds shm clean > /dev/null 2>&1

# The URDF through xacro, as robot_car.launch.py does, and in a params file
# as a quoted string: -p robot_description:=... is read as YAML, so a colon
# followed by a space anywhere in the URDF would break it
URDF=$(xacro "$REPO/urdf/robot_car.urdf.xml" robot_name:=robot_car)
PARAMS=$(mktemp --suffix=.yaml)
trap 'rm -f "$PARAMS"' EXIT
URDF=$URDF python3 -c '
import os, sys, yaml
yaml.safe_dump({"robot_state_publisher": {"ros__parameters": {"robot_description": os.environ["URDF"]}}}, sys.stdout)' > "$PARAMS"

python3 "$REPO/tools/sim_car.py" &
SIM=$!
# The binary itself, not through ros2 run: killing the ros2 run wrapper left
# robot_state_publisher running, publishing a stale TF on this domain
"$(ros2 pkg prefix robot_state_publisher)/lib/robot_state_publisher/robot_state_publisher" --ros-args \
  --params-file "$PARAMS" > /dev/null 2>&1 &
RSP=$!
trap 'kill $SIM $RSP 2>/dev/null; rm -f "$PARAMS"' EXIT
sleep 2

ros2 launch car_s pc_bringup.launch.py "$@"
