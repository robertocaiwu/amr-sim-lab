# Machine checks — runtime verification on a ROS 2 Humble + Gazebo Harmonic box

The whole project was developed and reviewed on a machine **without** ROS 2, Gazebo,
or Docker. Every pure-Python part is unit-tested (`make test` → 27 passing). Everything
that touches ROS or Gazebo was static-checked only (`python3 -m py_compile`, XML/YAML
parse, `flake8`). This file lists what still has to be verified on a real box, roughly
in dependency order. Tick them off as you go.

## 0. Build

- [ ] `docker build -f docker/Dockerfile -t amr-sim-lab .` succeeds.
      Known risk: `ros-humble-ros-gzharmonic-sim` in the Dockerfile is likely **not** a
      real apt package — drop it if `apt` errors (the real one, `ros-humble-ros-gz-sim`,
      is also listed). `scripts/install_ros_gazebo.sh` is the bare-metal alternative.
- [ ] `colcon build --symlink-install` from the repo root — all 4 packages build.
- [ ] `colcon test` — **must be green.** `test_spawn_integration` self-starts
      `spawner_only.launch.py` and skips if it can't come up in 40 s; the 3 packages
      run their pytest suites. The declared `ament_flake8` / `ament_pep257` test_depends
      are inert (no `test_flake8.py` files were generated) — not a blocker.

## 1. Interfaces & world

- [ ] `ros2 interface show scene_spawner_interfaces/srv/SpawnLayout` prints all
      request/response fields.
- [ ] `ros2 interface list | grep -i ros_gz` — confirm `ros_gz_interfaces/srv/SpawnEntity`
      and `ros_gz_interfaces/srv/DeleteEntity` exist with the fields the node assumes:
      `SpawnEntity.Request.entity_factory` (`.name`, `.sdf`, `.allow_renaming`);
      `DeleteEntity.Request.entity` (`.name`, `.type`) and `Entity.MODEL`.
      If they differ, adjust `_spawn_one` / `_remove_one` in `spawner_node.py`.
- [ ] `GZ_SIM_RESOURCE_PATH=$PWD/models gz sim -s -r --iterations 200 worlds/warehouse.sdf`
      — runs 200 steps, no plugin-load or parse errors. `<physics type="ignored">` may
      print a benign "using default physics engine" line.
- [ ] `gz sim worlds/warehouse.sdf` (GUI) shows the floor + four perimeter walls.

## 2. Models

- [ ] With the world running headless, spawn a cart directly:
      `gz service -s /world/warehouse/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 --req 'sdf: "<sdf version=\"1.10\"><include><name>cart_test</name><uri>model://cart</uri><pose>1 1 0 0 0 0</pose></include></sdf>"'`
      → `data: true`; `gz model --list` shows `cart_test`.
- [ ] Spawn `diffbot` (via `ros_gz_sim create` or `gz service`) — model loads, no plugin
      errors (`gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`,
      `gpu_lidar`). It should rest level on its two wheels + frictionless caster.

## 3. Bridge (the critical wiring — reviewed as broken pre-fix, fixed but unverified)

- [ ] `ros2 launch sim_bringup sim.launch.py` comes up: Gazebo GUI + warehouse + diffbot
      + ~8 carts.
- [ ] `ros2 service list | grep world` shows `/world/warehouse/create` and
      `/world/warehouse/remove` (the service-bridge entries added to `bridge.yaml`).
      **If they are absent**, this `ros_gz_bridge` build does not support service entries
      in the YAML — run a second `parameter_bridge` for services, or bridge them
      another way. Without this, every spawn returns "Gazebo create service unavailable".
- [ ] `gz topic -l | grep -Ei 'diffbot|scan|odom|cmd_vel|tf'` — confirm the real gz
      topic names. `bridge.yaml` assumes DiffDrive's **gz-default scoped** names
      (`/model/diffbot/cmd_vel`, `/model/diffbot/odometry`, `/model/diffbot/tf`) now
      that the `<topic>` overrides were removed from `models/diffbot/model.sdf`. Fix the
      `gz_topic_name` values in `bridge.yaml` if reality differs.
- [ ] `ros2 topic list` shows `/scan`, `/cmd_vel`, `/odom`, `/clock`, `/tf`.
- [ ] `ros2 topic echo /scan --once` returns a populated `LaserScan`.

## 4. Spawner node

- [ ] `ros2 run scene_spawner spawner_node --ros-args -p spawn_on_startup:=false` starts;
      `ros2 service list` shows `/scene_spawner/spawn_layout` + `/scene_spawner/clear_layout`.
- [ ] Startup spawn: with the full launch, ~8 carts appear around the robot.
- [ ] **Multi-cart spawn does not wedge.** The node calls Gazebo's create service from
      inside its own service callback, waiting on a `threading.Event` (not spinning) on a
      `MultiThreadedExecutor` + `ReentrantCallbackGroup`. Call
      `ros2 service call /scene_spawner/spawn_layout scene_spawner_interfaces/srv/SpawnLayout "{count: 12, seed: 7}"`
      — it should return `success: true, placed: 12` and the node stays responsive
      afterwards. If it hangs or the node goes unresponsive after the first entity,
      the `_call_sync` approach needs revisiting.
- [ ] Reproducibility: two `spawn_layout` calls with the same `seed` produce an
      identical layout.
- [ ] Invalid area: `spawn_layout` with `area_min_x == area_max_x` returns
      `success: false, message: "invalid area bounds", placed: 0` **and leaves any
      existing layout untouched** (not cleared).
- [ ] `ros2 service call /scene_spawner/clear_layout std_srvs/srv/Trigger {}` removes
      every cart; `gz model --list` shows 0 `cart_*`.

## 5. Safety-stop / teleop (end-to-end)

- [ ] `ros2 launch sim_bringup sim.launch.py teleop:=true` — an xterm opens with
      `teleop_twist_keyboard`.
- [ ] Driving forward, the robot moves (confirms `/cmd_vel_raw` → `safety_stop` →
      `/cmd_vel` → bridge → DiffDrive).
- [ ] Driving straight at a cart, the robot **halts ~0.6 m short** and can still rotate
      / reverse out. (`safety_stop` params: `stop_distance` 0.6, `front_arc_deg` 90.)
- [ ] Kill `/scan` (or launch without the robot) → with `require_scan:=true` the robot
      refuses to move (fail-safe).

## 6. Headless / GUI toggle

- [ ] `ros2 launch sim_bringup sim.launch.py gui:=false` runs headless (server only).
- [ ] `ros2 launch sim_bringup spawner_only.launch.py` runs headless and just brings up
      the world + spawner (used by the integration test).

## Known launch-arg limitation

`sim.launch.py` / `spawner_only.launch.py` accept a `world:=` arg, but the world **file**
loaded is hard-wired to `worlds/warehouse.sdf` and the service bridge hard-codes
`/world/warehouse/*`. The project is effectively single-world; `world:=other` would
desync. Remove the arg or generalise the bridge if multi-world is ever needed.
