#!/usr/bin/env bash
# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
set -euo pipefail
cd "$(dirname "$0")/.."
colcon build --symlink-install
# shellcheck disable=SC1091
source install/setup.bash
ros2 launch sim_bringup sim.launch.py &
LAUNCH_PID=$!
sleep 15
ros2 service call /scene_spawner/spawn_layout \
  scene_spawner_interfaces/srv/SpawnLayout "{count: 12, seed: 7}"
wait $LAUNCH_PID
