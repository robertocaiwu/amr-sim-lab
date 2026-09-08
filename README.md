# amr-sim-lab

A small, self-contained robotics simulation project built on **ROS 2 Humble** and
**Gazebo Harmonic**. It procedurally spawns **randomized cart layouts** into a
warehouse world — reproducible by seed and triggerable both at launch and at
runtime over a ROS 2 service — and brings up a drivable **`diffbot`** (a minimal
self-contained differential-drive AMR with a 2D lidar) that is gated by a
**safety-stop** node which halts it near obstacles. Everything runs from one
command via Docker or a VS Code devcontainer.

The project is an exploration of procedural scene generation for mobile-robot
simulation. It is deliberately seamed so that a **Phase 2** port of the same
ideas to NVIDIA Isaac Sim + OpenUSD + Replicator is a follow-up rather than a
rewrite.

![demo](docs/images/demo.gif)

## Architecture

Four packages in a single colcon workspace under `src/`:

| Package | Role |
|---|---|
| `scene_spawner_interfaces` | Interface-only package. Defines `srv/SpawnLayout.srv` (count / area / spacing / seed / model). |
| `scene_spawner` | The spawner. `layout_generator.py` is a pure, dependency-free deterministic placement algorithm; `sdf_templates.py` builds `<include>` spawn SDF; `spawner_node.py` is the ROS 2 node that offers `spawn_layout` / `clear_layout` and drives Gazebo's create/remove services. |
| `safety_stop` | `gate.py` is pure obstacle-gating logic (no ROS imports); `safety_stop_node.py` subscribes to `/scan` and `/cmd_vel_raw`, and republishes a gated `/cmd_vel` that blocks forward motion inside the stop distance. |
| `sim_bringup` | Launch + config. `sim.launch.py` starts Gazebo, the `ros_gz_bridge` parameter bridge, the robot, the spawner, safety-stop and optional teleop; `config/layout.yaml` is the canonical scene description; `config/bridge.yaml` maps the gz topics. |

Full design rationale: [`docs/superpowers/specs/2026-09-08-amr-sim-lab-design.md`](docs/superpowers/specs/2026-09-08-amr-sim-lab-design.md).

Runtime topic flow:

```
teleop_twist_keyboard ──▶ /cmd_vel_raw ──▶ safety_stop ──▶ /cmd_vel ──▶ ros_gz_bridge ──▶ gz diff-drive
                                              ▲
gz gpu_lidar ──▶ /scan ────────────────────────┘
```

## Requirements

- **Ubuntu 22.04**, **ROS 2 Humble**, and **Gazebo Harmonic**, installed via
  `ros-humble-ros-gzharmonic` from the OSRF apt repo.
- **or just Docker** (with the NVIDIA container toolkit for GPU rendering).

> **Humble ships with Fortress by default; this project uses Harmonic**, installed
> via `ros-humble-ros-gzharmonic` (which pulls `gz-harmonic` and the matching
> `ros_gz` bridge built against it). This is a documented, supported combination
> and is handled entirely inside `docker/Dockerfile`. It is the only environment
> subtlety in the project.

The `diffbot` model is first-party (`models/diffbot/`) — a plain SDF model with a
`gz-sim-diff-drive-system` plugin and a `gpu_lidar` sensor. There is no
submodule and no third-party robot to vendor.

## Quickstart (Docker)

```bash
docker build -f docker/Dockerfile -t amr-sim-lab .

xhost +local:docker   # allow the container to reach your X server

docker run -it --rm \
  --net=host --gpus all \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)":/ws \
  amr-sim-lab

# inside the container:
make run
```

VS Code users can instead "Reopen in Container" — `.devcontainer/devcontainer.json`
uses the same Dockerfile.

## Quickstart (native)

```bash
make build && make run
```

`make run` sources `install/setup.bash` and launches `sim.launch.py` with
`teleop:=true`.

## Usage

Launch directly:

```bash
ros2 launch sim_bringup sim.launch.py
```

Launch arguments:

| Arg | Default | Meaning |
|---|---|---|
| `world` | `warehouse` | World name; loads `worlds/<world>.sdf`. |
| `gui` | `true` | `true` runs the Gazebo GUI; `false` runs the server headless. |
| `teleop` | `false` | `true` opens `teleop_twist_keyboard` in an xterm. |
| `robot` | `diffbot` | Model name under `models/` to spawn as the AMR. |

On startup the spawner places `default_count` (8) carts using `startup_seed` (0).

**Spawn a new layout** (clears the current one first, so calls don't accumulate):

```bash
ros2 service call /scene_spawner/spawn_layout \
  scene_spawner_interfaces/srv/SpawnLayout "{count: 15, seed: 3}"
```

Full request fields: `count`, `area_min_x/y`, `area_max_x/y`, `min_spacing`,
`seed` (negative → non-deterministic), `model_name`. Any zero/empty field falls
back to the node default from `config/layout.yaml`. Same `seed` + same params →
identical layout. The response reports `requested`, `placed` and `entity_names`.

**Clear the layout:**

```bash
ros2 service call /scene_spawner/clear_layout std_srvs/srv/Trigger "{}"
```

**Teleop keys** (`teleop_twist_keyboard`, in its xterm window): `i` forward,
`,` back, `j`/`l` turn, `k` stop, `u`/`o`/`m`/`.` arc; `q`/`z` change max speed,
`w`/`x` linear only, `e`/`c` angular only. The safety-stop node will zero forward
velocity whenever the lidar sees an obstacle within `stop_distance` (0.6 m) in
the front 90° arc; rotation and reverse still pass through.

## Testing

```bash
make test              # pure unit tests — no ROS graph, just the Python packages
make test-integration  # builds the workspace, runs the sim_bringup launch test
```

`make test` runs `pytest` in `src/scene_spawner` and `src/safety_stop`. These
cover `layout_generator` (determinism, spacing, saturation, invalid area), the
SDF template builder, the safety gate, and a guard test asserting the pure
modules import no ROS packages. `make test-integration` needs a sourced,
built workspace and a working Gazebo Harmonic install.

## Phase 2 roadmap — NVIDIA Isaac Sim

Phase 2 ports the same procedural-scene idea to **Isaac Sim + OpenUSD +
Replicator** without rewriting the core:

- Reuse `scene_spawner/layout_generator.py` and `config/layout.yaml` unchanged as
  the scene source of truth — they are pure and simulator-agnostic.
- Convert `models/cart` to USD (single-material geometry → one-liner conversion)
  and assemble the warehouse as a USD stage with references.
- Drive `diffbot` (URDF → USD import) over the **Isaac Sim ROS 2 bridge**
  (OmniGraph action graph) with the same `/cmd_vel` + `/scan` contract.
- Replace launch-time spawning with a **Replicator** randomizer that reads the
  same layout params and writes a few hundred annotated frames.
- Add one **Isaac Lab** RL task on the GPU-vectorized environment.

Concept-by-concept mapping: [`docs/gazebo-vs-isaac.md`](docs/gazebo-vs-isaac.md).

## License

Apache-2.0 — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE). This is an
independent project; all models, worlds, and code are first-party.
