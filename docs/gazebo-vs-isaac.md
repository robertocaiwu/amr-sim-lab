# Gazebo Harmonic vs NVIDIA Isaac Sim — a living comparison

Working notes for the Phase 2 port. Records the concept mapping and the decisions
as they are made. Not exhaustive; corrected as understanding improves.

## Scene description: SDF vs USD

- **Gazebo** describes a scene as **SDF**: a `<world>` contains `<model>` elements,
  each with `<link>`/`<joint>`/`<sensor>`, and `<include>` pulls a model by URI
  (`model://cart`) from the resource path. It is a flat XML tree resolved at load.
- **Isaac / OpenUSD** describes a scene as a **stage** of **prims** arranged in a
  namespace hierarchy. Geometry, transforms and physics are **attributes** on
  prims (or on applied API schemas).
- USD **references** and **payloads** are the analogue of `<include>`, but
  composition is layered: **sublayers**, **variant sets**, and over-prims let a
  stronger layer override attributes of a weaker one without editing it.
- Porting `worlds/warehouse.sdf`: the static warehouse becomes a base USD layer;
  each spawned cart becomes a referenced prim added on a session/override layer,
  which is a cleaner match for "clear and re-spawn" than deleting SDF entities.

## Physics: DART (CPU) vs PhysX (GPU)

- **Gazebo Harmonic** defaults to the **DART** physics engine, stepped on the
  **CPU** at a fixed timestep, typically targeting real-time factor ~1.0.
- **Isaac Sim** uses **PhysX 5** with **GPU** rigid-body and contact solving,
  designed to simulate many actors and many environments in parallel.
- Determinism differs: DART at a fixed step is repeatable; PhysX GPU is fast but
  its parallel solver requires care to reproduce a run bit-for-bit.
- For this project the physics contract is small (a diff-drive base plus static
  box carts), so the cart dimensions and masses transfer directly; only the
  friction/contact tuning needs a re-check under PhysX.

## Extensibility: gz-sim systems vs Omniverse Kit extensions

- **Gazebo** extends behavior with **systems**: shared libraries attached via
  `<plugin>` to a world or model (e.g. `gz-sim-diff-drive-system`,
  `gz-sim-joint-state-publisher-system`). Each runs per-entity in the server loop.
- **Isaac Sim** is a **Kit application**: functionality is packaged as **Kit
  extensions** (Python or C++) that register menus, OmniGraph nodes, and update
  callbacks against the USD stage.
- Roughly: a gz system ≈ an extension that subscribes to the physics step; a
  bridge topic map ≈ an OmniGraph action graph.
- The `diffbot` diff-drive plugin has a direct Isaac equivalent
  (a differential controller graph node), so the robot's control surface is
  portable.

## Sensors & synthetic data: gz sensors + bridge vs Isaac Replicator

- **Gazebo** sensors (`gpu_lidar`, `camera`, `imu`, ...) are declared in SDF and
  publish on gz transport; the ROS side sees them only through the bridge.
- **Isaac** renders sensors on the GPU (RTX lidar, camera) and adds **Replicator**:
  a graph of **annotators** (depth, segmentation, bounding boxes), **writers**
  (to disk in COCO/KITTI/USD forms), and **randomizers** (pose, materials, light).
- Replicator makes the spawner's task — randomized layouts — a first-class,
  offline dataset-generation pipeline rather than a runtime service call.
- `SpawnLayout.srv` (count / area / spacing / seed) maps cleanly onto Replicator
  randomizer parameters.

## ROS bridge: `ros_gz_bridge` vs Isaac Sim ROS 2 bridge

- **Gazebo** uses `ros_gz_bridge`'s **`parameter_bridge`**, configured here by
  `config/bridge.yaml` — a YAML list of `{ros_topic_name, gz_topic_name,
  ros_type_name, gz_type_name, direction}` entries, one process bridging gz
  transport ↔ DDS.
- **Isaac Sim** uses the **Isaac Sim ROS 2 bridge** extension: publishers and
  subscribers are **OmniGraph nodes** wired in an action graph on the stage,
  reading/writing USD attributes directly.
- Both terminate at the same ROS 2 topic contract (`/cmd_vel`, `/scan`, `/odom`,
  `/tf`, `/clock`), so `safety_stop` and any downstream node are unchanged.
- The gz bridge is declarative and external; the Isaac graph is in-stage and
  visual — more setup, but no separate process and no type-name bookkeeping.

## Scale: one world vs Isaac Lab vectorized envs

- **Gazebo** runs **one world** at (near) real time. Parallelism means launching
  multiple simulator processes, each its own DDS graph.
- **Isaac Lab** clones an environment into **thousands of vectorized copies** on a
  single GPU, stepping them as one batched tensor operation for RL throughput.
- The procedural layout generator is the natural bridge: instead of one seeded
  layout it produces one per env index, feeding a batched reset.

## What ports directly from this repo

- **`scene_spawner/layout_generator.py`** — pure Python, no ROS/Gazebo/I/O; the
  placement algorithm is reused verbatim.
- **`config/layout.yaml`** — the canonical scene description; Phase 2 tooling
  reads the same file.
- **`SpawnLayout.srv` semantics** — count / area / spacing / seed / placed become
  the parameter surface for a Replicator randomizer or a Kit extension.
- **Cart dimensions and mass** from `models/cart/model.sdf` — carried into the USD
  cart asset unchanged.
- **The ROS 2 topic contract** — `safety_stop` and its `gate.py` logic are
  simulator-agnostic and move over untouched.
