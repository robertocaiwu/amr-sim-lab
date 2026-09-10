# amr-sim-lab — Design

**Date:** 2026-09-08
**Status:** Approved for planning
**Repo:** `amr-sim-lab` (root: `/workspace/simulation`)
**License:** Apache-2.0

> **Implementation note (added 2026-09-09, during execution):** three design
> points changed during implementation; the sections below are not yet
> rewritten to match:
> - **Robot:** the spec calls for Dolly (§2, §4.3). Dolly's maintained branch
>   targets ROS 2 Galactic / Ignition Fortress and does not load on Gazebo
>   Harmonic, so the implementation uses a self-contained `models/diffbot/`
>   SDF (differential drive + `gpu_lidar`) — the fallback §2 anticipated. No
>   third-party robot is vendored; there is no submodule.
> - **Cart mesh:** the cart (§4.2) uses primitive box/cylinder geometry, not a
>   glTF mesh. USD conversion for Phase 2 still works with primitives.
> - **NOTICE:** carries no third-party attribution (nothing is vendored).

---

## 1. Purpose

A small, public, well-packaged robotics simulation project that:

1. Procedurally spawns **randomized cart layouts** into a warehouse world — reproducible by seed, triggerable both at launch and at runtime via a ROS 2 service.
2. Brings up an **off-the-shelf AMR** (Dolly) that is driven with `/cmd_vel` + keyboard teleop, carries a **2D lidar**, and is gated by a small **safety-stop** node that halts it near obstacles.
3. Ships a **Dockerfile + devcontainer** so the project runs with one command on a fresh machine.

The project is a self-contained exploration of procedural scene generation for mobile-robot simulation. It is structured so a **Phase 2** port of the same ideas to NVIDIA Isaac Sim + OpenUSD + Replicator is a small, well-seamed follow-up rather than a rewrite.

### Non-goals

- No autonomous navigation (no Nav2, no path planning). The AMR is drivable, not autonomous.
- No multi-*robot* coordination. "Multiple models" means multiple carts; there is exactly one AMR.
- No CI in this phase (Dockerfile + devcontainer only).
- Phase 2 (Isaac Sim) is **scoped here but not implemented**. Only the seams are designed in.

---

## 2. Target environment

| Component | Version / choice | Notes |
|---|---|---|
| OS | Ubuntu 22.04 | Developer machine has an NVIDIA RTX GPU |
| ROS 2 | Humble Hawksbill | LTS |
| Gazebo | Harmonic (`gz-sim` 8) | **Not** Humble's default (Fortress). Requires `ros-humble-ros-gzharmonic` from the OSRF apt repo — a documented, supported combination. |
| Robot | Dolly | Gazebo's own demo robot: Harmonic-native, Apache-2.0, ships with a 2D lidar, designed for lidar-reactive behavior. Swappable via a launch argument. |
| Python | 3.10 | System Python on 22.04 |
| Container | Docker + VS Code devcontainer | Base image `ros:humble`, Gazebo Harmonic layered on top |

### Dependency on the Humble + Harmonic pairing

Humble ships with Gazebo Fortress by default. Harmonic is installed by adding the OSRF/`packages.osrfoundation.org` apt repo and installing `ros-humble-ros-gzharmonic` (which pulls `gz-harmonic` and the matching `ros_gz` bridge built against it). This is the only environment subtlety in the project and is handled entirely inside the Dockerfile. The README states it explicitly.

If Harmonic ever proves too troublesome on Humble, the fallback (documented, not built) is Fortress + `ros-humble-ros-gz` with the same package layout — `layout_generator`, `safety_stop`, and the srv are all Gazebo-version-agnostic; only `scene_spawner`'s spawn/remove service names and `sim_bringup`'s bridge config change.

---

## 3. Architecture

Four packages in a single colcon workspace under `src/`.

### 3.1 `scene_spawner_interfaces`

Interface-only package (`rosidl` generation, no Python/C++ logic).

**`srv/SpawnLayout.srv`**

```
# Request
int32   count            # number of carts to place; 0 => use node default
float64 area_min_x        # grid bounds (metres, world frame)
float64 area_min_y
float64 area_max_x
float64 area_max_y
float64 min_spacing       # minimum centre-to-centre distance; 0 => use node default
int32   seed              # RNG seed; negative => nondeterministic (time-seeded)
string  model_name        # spawnable model to use; "" => use node default ("cart")
---
# Response
bool          success
string        message
int32         requested   # carts asked for
int32         placed       # carts actually placed (< requested if area is saturated)
string[]      entity_names # names of the spawned entities, for later removal
```

`/clear_layout` uses `std_srvs/srv/Trigger` (no custom type needed).

### 3.2 `scene_spawner`

The spawner node plus a **pure, dependency-free** layout module.

**`scene_spawner/layout_generator.py`** — pure Python, no ROS, no Gazebo imports.

```python
@dataclass(frozen=True)
class LayoutParams:
    count: int
    area: tuple[float, float, float, float]   # (min_x, min_y, max_x, max_y)
    min_spacing: float
    footprint_radius: float                   # cart bounding radius, for overlap rejection
    yaw_jitter: bool = True                    # random heading per cart
    seed: int | None = None

@dataclass(frozen=True)
class Placement:
    name: str          # e.g. "cart_03"
    x: float
    y: float
    yaw: float

def generate_layout(params: LayoutParams) -> list[Placement]:
    """Rejection-sample non-overlapping cart poses inside `area`.

    Deterministic for a given integer seed. Places as many carts as fit;
    if `count` cannot be satisfied after MAX_ATTEMPTS_PER_CART tries per
    cart, returns the partial list (caller reports requested vs placed).
    """
```

- Algorithm: uniform random sampling in `area`, reject a candidate if it is within `min_spacing` (or `2 * footprint_radius`, whichever is larger) of an already-placed cart. `MAX_ATTEMPTS_PER_CART = 100`. Stop early when a cart cannot be placed and return the partial list.
- RNG: a local `random.Random(seed)` instance — never the global `random` module — so results are independent of other code and fully reproducible.
- No file I/O, no logging side effects. Returns data.

**`scene_spawner/spawner_node.py`** — the ROS 2 node.

- Parameters (declared, overridable by launch / CLI / `layout.yaml`):
  - `world_name` (str, default `"warehouse"`)
  - `default_count` (int, default `8`)
  - `default_area` (float[4], default `[-4.0, -4.0, 4.0, 4.0]`)
  - `default_min_spacing` (float, default `1.0`)
  - `default_model` (str, default `"cart"`)
  - `cart_footprint_radius` (float, default `0.5`)
  - `spawn_on_startup` (bool, default `true`)
  - `startup_seed` (int, default `0`)
- Services provided:
  - `~/spawn_layout` (`scene_spawner_interfaces/SpawnLayout`) — clears the current layout, generates a new one via `generate_layout`, spawns each placement.
  - `~/clear_layout` (`std_srvs/Trigger`) — removes every entity this node has spawned.
- Gazebo interaction: calls the Gazebo Transport services the `ros_gz_sim` bridge exposes —
  - spawn: `/world/<world_name>/create` (`ros_gz_interfaces/srv/SpawnEntity`) with an SDF string built from a template + the model's mesh/collision, or an `<include>` of the installed `cart` model.
  - remove: `/world/<world_name>/remove` (`ros_gz_interfaces/srv/DeleteEntity`).
  - The node waits (bounded, `create_timeout_sec` param, default 10 s) for `/world/<world_name>/create` to be available before serving requests or doing the startup spawn.
- Startup spawn: if `spawn_on_startup`, after the create service is available, call the same code path as `/spawn_layout` with the node defaults and `startup_seed`.
- State: keeps `self._spawned: list[str]` (entity names). `clear_layout` removes them and empties the list. `spawn_layout` clears first, so repeated calls don't accumulate carts.

### 3.3 `safety_stop`

**`safety_stop/gate.py`** — pure function.

```python
def gate_twist(scan_ranges: Sequence[float], range_min: float, range_max: float,
               cmd: TwistLike, stop_distance: float,
               front_arc_rad: float) -> TwistLike:
    """Return `cmd` unchanged, or with linear.x clamped to <= 0 when the
    closest valid return inside the front arc is nearer than stop_distance.
    Angular velocity and reverse motion are always passed through."""
```

- Only forward motion (`linear.x > 0`) is blocked; the robot can always rotate and reverse out of trouble.
- Invalid returns (NaN, inf, `< range_min`, `> range_max`) are ignored.
- `front_arc_rad` limits the check to a cone around the robot's heading so carts beside/behind it don't freeze it.

**`safety_stop/safety_stop_node.py`**

- Subscribes: `/cmd_vel_raw` (`geometry_msgs/Twist`), `/scan` (`sensor_msgs/LaserScan`).
- Publishes: `/cmd_vel` (`geometry_msgs/Twist`).
- Parameters: `stop_distance` (default `0.6`), `front_arc_deg` (default `90`), `require_scan` (default `true`), `scan_timeout_sec` (default `1.0`).
- Behavior:
  - On each `/cmd_vel_raw`, apply `gate_twist` using the most recent scan and publish to `/cmd_vel`.
  - If `require_scan` and no scan has arrived yet, or the last scan is older than `scan_timeout_sec`, publish a **zero** Twist (fail-safe) and log a throttled warning.
  - If `require_scan` is `false` and there is no scan, pass `/cmd_vel_raw` through unchanged.

### 3.4 `sim_bringup`

Launch + configuration only. No nodes of its own.

- `launch/sim.launch.py` — the full demo:
  1. `gz sim` with `worlds/warehouse.sdf` (headless togglable via `gui:=false`).
  2. `ros_gz_bridge` with `config/bridge.yaml`.
  3. Spawn the robot (`ros_gz_sim create` from the Dolly SDF, or `robot:=<name>` to swap).
  4. `scene_spawner` node, parameters from `config/layout.yaml`.
  5. `safety_stop` node.
  6. `teleop_twist_keyboard` (only when `teleop:=true`, in its own xterm; default `false` so the launch is non-interactive for testing).
- `launch/spawner_only.launch.py` — gz + bridge + `scene_spawner` only, headless, for the integration test and for iterating on layouts.
- `config/bridge.yaml` — bridges `/clock`, `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/tf_static` between ROS and Gazebo Transport (directions and types spelled out).
- `config/layout.yaml` — the shared, stack-agnostic scene definition:

```yaml
scene_spawner:
  ros__parameters:
    world_name: warehouse
    default_count: 8
    default_area: [-4.0, -4.0, 4.0, 4.0]
    default_min_spacing: 1.0
    default_model: cart
    cart_footprint_radius: 0.5
    spawn_on_startup: true
    startup_seed: 0
```

---

## 4. Assets

### 4.1 `worlds/warehouse.sdf`

Gazebo Harmonic (SDF 1.10) world:

- Systems: `Physics` (DART), `UserCommands`, `SceneBroadcaster`, `Sensors` (for the lidar), `Contact`.
- A ground plane and a rectangular perimeter of low walls (~8 m × 8 m interior) so the AMR and carts stay contained.
- One directional light + ambient, tuned so screenshots/GIFs read well.
- No robot and no carts baked in — both are spawned at launch.

### 4.2 `models/cart/`

- `model.sdf` + `model.config`, a Gazebo model named `cart`.
- Geometry: a **glTF (`.glb`) mesh** for visuals (kept clean and single-material so USD conversion in Phase 2 is trivial), a simple box collision.
- **Dynamic rigid body** (not static), no joints, no plugins. Realistic mass (~20 kg) and surface friction so a cart stays where it is spawned but can be bumped by the AMR — this behaves sensibly if the safety-stop is disabled.
- Installed to the package share dir and added to `GZ_SIM_RESOURCE_PATH` by the launch files.

### 4.3 Robot

Dolly is vendored as a git submodule or copied under `models/` (Apache-2.0, attribution in `NOTICE`). Its lidar is remapped/bridged to `/scan`. A `robot:=` launch arg selects which model SDF to spawn, so swapping in another AMR later touches only launch args + `bridge.yaml`.

---

## 5. Data flow

```
ros2 launch sim_bringup sim.launch.py
  │
  ├─ gz sim warehouse.sdf ────────────── Gazebo Transport
  ├─ ros_gz_bridge (bridge.yaml) ─────── /clock /cmd_vel /odom /scan /tf
  ├─ create (Dolly) ─────────────────── robot in world
  ├─ scene_spawner ──── reads layout.yaml
  │      └─ (spawn_on_startup) generate_layout(seed=0) ─► /world/warehouse/create ×N ─► carts
  └─ safety_stop

runtime re-roll:
  ros2 service call /scene_spawner/spawn_layout scene_spawner_interfaces/srv/SpawnLayout "{count: 12, seed: 7}"
    └─ clear_layout (remove previous) ─► generate_layout(seed=7) ─► create ×N

driving:
  teleop_twist_keyboard ─► /cmd_vel_raw
  safety_stop( /cmd_vel_raw, /scan ) ─► /cmd_vel ─► bridge ─► gz ─► robot moves
  robot lidar ─► gz ─► bridge ─► /scan ─► safety_stop
```

---

## 6. Error handling

| Situation | Behavior |
|---|---|
| `/world/<world>/create` not available yet | `scene_spawner` waits up to `create_timeout_sec` (10 s). If it never appears: log error, skip startup spawn, still start the services; each `spawn_layout` call then retries once and returns `success=false, message="Gazebo create service unavailable"`. |
| Area saturated (can't fit `count` carts) | `generate_layout` returns a partial list. Response: `success=true`, `placed < requested`, `message` notes the shortfall. Not treated as an error. |
| `spawn_layout` called with `count <= 0` | Use `default_count`. |
| Invalid area (min >= max) | Response `success=false, message="invalid area bounds"`, nothing spawned. |
| `seed < 0` | Nondeterministic (time-seeded) layout; documented, not an error. |
| No `/scan` yet, `require_scan=true` | `safety_stop` publishes zero Twist (robot immobile) + throttled warning until the first scan. |
| `/scan` stale (> `scan_timeout_sec`) | Same fail-safe (zero Twist). |
| `require_scan=false`, no `/scan` | `/cmd_vel_raw` passes through unchanged. |
| `clear_layout` with nothing spawned | `success=true`, `message="nothing to clear"`. |
| Determinism | Same non-negative seed ⇒ identical `Placement` list. Asserted by unit test. |

---

## 7. Testing

### Unit (pytest, no ROS graph)

- `scene_spawner/test/test_layout_generator.py`
  - same seed ⇒ identical output (determinism)
  - different seed ⇒ different output
  - every placement inside `area`
  - no two placements closer than `min_spacing`
  - over-capacity request ⇒ `len(result) < count`, no exception
  - `count=0` ⇒ empty list
- `safety_stop/test/test_gate.py`
  - clear path ⇒ `cmd` unchanged
  - obstacle inside `stop_distance` and front arc ⇒ `linear.x` clamped to `<= 0`
  - obstacle behind / outside arc ⇒ `cmd` unchanged
  - obstacle close but `cmd.linear.x <= 0` (reversing) ⇒ unchanged
  - NaN/inf/out-of-range returns ⇒ ignored

### Integration (`launch_testing`, local only — documented `make test-integration`)

- `test/test_spawn_integration.py`
  - launch `spawner_only.launch.py` headless (`gz sim -s -r`)
  - wait for `/scene_spawner/spawn_layout`
  - call it with `count=5, seed=1`
  - assert response `placed == 5`
  - query the Gazebo scene (`gz model --list` or the scene-info service) and assert 5 `cart_*` entities present
  - call `/clear_layout`, assert 0 remain

### Not automated

- Visual / teleop driving is manual (documented smoke-test checklist in `docs/`).
- CI is out of scope this phase.

---

## 8. Phase 2 seams (Isaac Sim + OpenUSD + Replicator — not built now)

The design keeps the port small:

| Seam | How it helps Phase 2 |
|---|---|
| `layout_generator.py` is pure Python, zero ROS/Gazebo imports | An Isaac Sim Replicator script `import`s `generate_layout` directly and places USD prims at the returned poses — the randomization logic is shared, not reimplemented. |
| `config/layout.yaml` is a plain scene description, not Gazebo-specific | Same file drives the Isaac scene. |
| `models/cart` visual is a clean single-material `.glb` | `.glb → .usd` conversion is a one-liner; no mesh cleanup needed. |
| `SpawnLayout.srv` semantics (count/area/seed/placed) | Reused as the parameter surface for a Replicator randomizer or an Omniverse extension. |
| `docs/gazebo-vs-isaac.md` (started in this phase) | Running comparison notes: SDF↔USD stage/prim/attribute model, DART↔PhysX/GPU physics, gz systems↔Kit extensions, single world↔Isaac Lab vectorized envs, `ros_gz_bridge`↔Isaac Sim ROS 2 bridge. Records the mapping decisions as they are made. |

Phase 2 deliverables (for a later spec): warehouse USD stage, Replicator pose/lighting randomization writing a few hundred annotated frames, URDF→USD robot import driven over the Isaac Sim ROS 2 bridge, one Isaac Lab RL task, GPU-scale architecture write-up.

---

## 9. Repository layout

```
/workspace/simulation/                 # repo: amr-sim-lab
├── README.md                          # what it is; quickstart; GIF; architecture diagram; Phase 2 (Isaac Sim) roadmap
├── LICENSE                            # Apache-2.0
├── NOTICE                             # attribution: Dolly, any vendored assets
├── .gitignore                         # build/, install/, log/, __pycache__, *.pyc
├── .devcontainer/devcontainer.json    # points at docker/Dockerfile, mounts workspace, ROS env
├── docker/
│   ├── Dockerfile                     # ros:humble + Gazebo Harmonic + ros-humble-ros-gzharmonic + deps
│   └── entrypoint.sh                  # source /opt/ros + install/setup.bash
├── docs/
│   ├── superpowers/specs/2026-09-08-amr-sim-lab-design.md   # this file
│   ├── gazebo-vs-isaac.md             # Phase 2 comparison notes (living doc)
│   ├── smoke-test.md                  # manual teleop / visual checklist
│   └── images/                        # architecture diagram, screenshots
├── worlds/warehouse.sdf
├── models/
│   └── cart/{model.sdf, model.config, meshes/cart.glb}
├── config/
│   └── layout.yaml
├── scripts/
│   └── demo.sh                        # colcon build && ros2 launch ... && sample service call
├── Makefile                           # build, test, test-integration, run, clean
└── src/
    ├── scene_spawner_interfaces/
    │   ├── srv/SpawnLayout.srv
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── scene_spawner/
    │   ├── scene_spawner/{__init__.py, layout_generator.py, spawner_node.py, sdf_templates.py}
    │   ├── test/{test_layout_generator.py, test_flake8.py, test_pep257.py}
    │   ├── launch/                     # (none; bringup owns launch)
    │   ├── resource/scene_spawner
    │   ├── setup.py, setup.cfg, package.xml
    ├── safety_stop/
    │   ├── safety_stop/{__init__.py, gate.py, safety_stop_node.py}
    │   ├── test/{test_gate.py, test_flake8.py}
    │   ├── resource/safety_stop
    │   ├── setup.py, setup.cfg, package.xml
    └── sim_bringup/
        ├── launch/{sim.launch.py, spawner_only.launch.py}
        ├── config/{bridge.yaml, layout.yaml}
        ├── setup.py, setup.cfg, package.xml
```

`config/layout.yaml` is the canonical copy; `sim_bringup/config/layout.yaml` is a symlink or a build-time copy (decision for the plan — symlink preferred).

Apache-2.0 license header block at the top of every source file (`.py`, `.launch.py`, `CMakeLists.txt`); the scaffold adds them once.

---

## 10. Open items for the implementation plan

- Exact Dockerfile apt incantation for Gazebo Harmonic on Humble (OSRF repo key + `ros-humble-ros-gzharmonic`), pinned where possible.
- Whether to vendor Dolly as a submodule or a copied subtree (submodule preferred for clean attribution; copy if the upstream layout fights the colcon build).
- glTF cart mesh: author a simple one in Blender vs. source an Apache/CC0 warehouse-cart model.
- `layout.yaml` single-source: symlink vs. `setup.py` data-files copy.
- README GIF capture workflow (asciinema/peek) — nice-to-have, can slip.
