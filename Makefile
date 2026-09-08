# Copyright 2026 amr-sim-lab contributors. Apache-2.0. See LICENSE.
.PHONY: build test test-integration run clean

build:
	colcon build --symlink-install

test:
	cd src/scene_spawner && python3 -m pytest test/ -v
	cd src/safety_stop && python3 -m pytest test/ -v

test-integration:
	colcon build --symlink-install
	. install/setup.bash && python3 -m pytest src/sim_bringup/test/ -v

run:
	. install/setup.bash && ros2 launch sim_bringup sim.launch.py teleop:=true

clean:
	rm -rf build install log
