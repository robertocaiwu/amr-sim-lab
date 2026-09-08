# Manual smoke test

A quick end-to-end check that the simulation is healthy. Run it after a build,
after a dependency change, or before recording the demo. Automated unit tests
(`make test`) cover the pure logic; this checklist covers the parts that need a
running Gazebo and a human at the keyboard.

Prerequisites: Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic (or the Docker
image), an X server for the GUI.

## 1. Build

```bash
make build
. install/setup.bash
```

- [ ] `colcon build --symlink-install` finishes with no failed packages.
- [ ] `ros2 pkg list | grep -E 'scene_spawner|safety_stop|sim_bringup'` shows all
      four packages.

## 2. Launch

```bash
ros2 launch sim_bringup sim.launch.py
```

- [ ] The Gazebo GUI opens and shows the `warehouse` world.
- [ ] The `diffbot` AMR is present near the origin, with its lidar rays visible.
- [ ] Roughly **8 carts** appear shortly after startup (the on-launch spawn with
      `default_count: 8`, `startup_seed: 0`).
- [ ] `ros2 topic list` shows `/scan`, `/cmd_vel`, `/cmd_vel_raw`, `/odom`,
      `/tf`, `/clock`.
- [ ] `ros2 topic echo /scan --once` prints a `LaserScan` with finite ranges.

## 3. Deterministic re-spawn

```bash
ros2 service call /scene_spawner/spawn_layout \
  scene_spawner_interfaces/srv/SpawnLayout "{count: 15, seed: 3}"
# ...note the cart positions, then run the exact same call again:
ros2 service call /scene_spawner/spawn_layout \
  scene_spawner_interfaces/srv/SpawnLayout "{count: 15, seed: 3}"
```

- [ ] First call returns `success: true` and the previous (startup) carts are
      gone — replaced, not added to.
- [ ] `placed` is 15 (or fewer with a message about saturation if the area is
      tight — should be 15 for the default 8×8 m area).
- [ ] The second call produces a **visually identical** layout — same positions,
      same yaws.
- [ ] A call with a different seed (e.g. `{count: 15, seed: 4}`) produces a
      **different** layout.

## 4. Safety-stop

```bash
# relaunch with teleop, or: make run
ros2 launch sim_bringup sim.launch.py teleop:=true
```

- [ ] An xterm opens running `teleop_twist_keyboard`.
- [ ] Pressing `i` drives the robot forward; `j`/`l` turn; `k` stops.
- [ ] Drive straight at a cart. As the front lidar arc reads closer than
      `stop_distance` (0.6 m), the robot **halts** even while `i` is held.
- [ ] While halted against the cart, `j`/`l` still rotate and `,` still reverses
      (only forward is gated).
- [ ] Back away and the robot moves forward again.

## 5. Clear

```bash
ros2 service call /scene_spawner/clear_layout std_srvs/srv/Trigger "{}"
```

- [ ] Returns `success: true` with a `removed N entities` message.
- [ ] Every cart disappears from the world; the robot remains.
- [ ] A second call returns `success: true`, `message: "nothing to clear"`.

## 6. Shutdown

- [ ] `Ctrl-C` in the launch terminal stops every node and closes Gazebo cleanly.
- [ ] `make clean` removes `build/`, `install/`, `log/`.
