#!/usr/bin/env bash
# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
set -e
source /opt/ros/humble/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
