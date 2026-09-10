# Porting amr-sim-lab to NVIDIA Isaac Sim (Phase 2)

A step-by-step guide to reproducing the current Gazebo demo — a walled warehouse,
a seeded procedurally-placed batch of carts, and a drivable lidar robot with an
obstacle safety-stop — inside NVIDIA Isaac Sim.

## The load-bearing idea

The project was seamed so that most of it moves without change:

| Ports **unchanged** | Must be **rebuilt** for Isaac |
|---|---|
| `src/scene_spawner/scene_spawner/layout_generator.py` (pure stdlib) | `worlds/warehouse.sdf` → a USD stage |
| `config/layout.yaml` (the scene definition) | `models/cart/model.sdf` → `cart.usda` |
| the whole `src/safety_stop/` package (pure ROS 2) | `models/diffbot/model.sdf` → URDF → USD |
| `src/scene_spawner_interfaces/` (`SpawnLayout.srv`), if you keep the service API | the "spawn" mechanism (`gz service` → USD stage edits) |
| | the ROS bridge (`bridge.yaml` → an OmniGraph action graph) |

Proving those two seams — the layout generator produces the *identical* layout for
a given seed, and the unmodified `safety_stop_node` gives *identical* behaviour —
is the headline result of this port.

## Suggested layout

Add an `isaac/` tree at the repo root; leave the ROS packages where they are.

```
isaac/
├── README.md
├── assets/
│   ├── warehouse.usda
│   ├── cart.usda
│   └── diffbot/{diffbot.urdf, diffbot.usd}
├── scripts/{hello.py, spawn_layout.py, drive_test.py, replicator_carts.py}
├── graphs/ros2_bridge.md          # screenshot + node list of the action graph
└── out/                           # gitignored — synthetic-data output
```

## Order of work

Phases **0 → 1 → 2 → 3** give the reproducible warehouse + carts and the seam
proof. **4 → 6** add the drivable robot and the `safety_stop` reuse. **7** is the
synthetic-data story. **8** is an optional stretch. Do not start Phase 8 before the
job application goes out.

---

## Phase 0 — Environment

**0.1 GPU.** `nvidia-smi` — Isaac Sim needs an RTX card, ≥8 GB VRAM (RTX 3070 floor,
4080+ comfortable). No RTX card → rent a cloud GPU for a weekend (RunPod / Vast.ai
`RTX 4090`, or AWS `g5.2xlarge`), roughly €5–15 total. The **Learn OpenUSD** DLI
courses run in-browser with no GPU — do those in parallel.

**0.2 Install Isaac Sim.** Pick one:
- **pip (recommended, 4.2+):**
  ```bash
  python3.10 -m venv isaacsim_env && source isaacsim_env/bin/activate
  pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
  isaacsim            # launches the app
  ```
- **Container:** `docker pull nvcr.io/nvidia/isaac-sim:4.5.0` — best for headless /
  cloud / Replicator batch runs.
- **Omniverse Launcher:** deprecated; skip.

**0.3 Extension naming.** Isaac Sim 4.5 (Feb 2025) renamed the Python extensions.
Pick your version first, then use the matching names throughout:

| ≤ 4.2 | 4.5+ |
|---|---|
| `omni.isaac.core` | `isaacsim.core.api` |
| `omni.isaac.ros2_bridge` | `isaacsim.ros2.bridge` |
| `omni.isaac.sensor` | `isaacsim.sensors.rtx` / `isaacsim.sensors.physics` |
| `omni.importer.urdf` | `isaacsim.asset.importer.urdf` |
| `omni.isaac.wheeled_robots` | `isaacsim.robot.wheeled_robots` |

This guide uses the 4.5 names.

**0.4 Smoke test** — `isaac/scripts/hello.py`:
```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})
from isaacsim.core.api import World
world = World(); world.scene.add_default_ground_plane(); world.reset()
for _ in range(200):
    world.step(render=True)
app.close()
```
Run: `python isaac/scripts/hello.py` (inside the venv, or `./python.sh` in the
container). **Verify:** a window opens with a grid floor; the script exits with no
import errors.

---

## Phase 1 — The warehouse stage in USD

Recreate `worlds/warehouse.sdf`: an 8×8 m room (walls at ±4.1 m, 0.5 m tall), a
floor, one directional + one dome light.

**1.1** Author `isaac/assets/warehouse.usda`:
- `/World` — Xform, default prim
- `/World/ground` — `Plane` (or a flat `Cube`) + `UsdPhysics.CollisionAPI`, static
  (**no** `RigidBodyAPI`)
- `/World/walls/{north,south,east,west}` — `Cube` prims `8.4 × 0.2 × 0.5`,
  translated to `(0, ±4.1, 0.25)` / `(±4.1, 0, 0.25)`, `CollisionAPI` only
- `/World/sun` — `DistantLight`, intensity ~1000, angled
- `/World/sky` — `DomeLight`, intensity ~300

If hand-writing `.usda` is unfamiliar: build it in the GUI (Create → Mesh → Cube,
set transforms in the Property panel, Physics → Add → Colliders), Save As
`warehouse.usda`, then read the text to learn the schema.

**1.2 Verify:** open `warehouse.usda`, press Play — nothing moves, no console
errors. Drop a cube from 2 m → it lands on the floor and is stopped by a wall.

**Docs:** *Learn OpenUSD → Stages, Prims, Attributes*; Isaac Sim → *Rigid Body
Simulation*, the `UsdPhysics` schema reference.

---

## Phase 2 — The cart in USD

`models/cart/model.sdf` is a `0.8 × 0.5 × 0.1` deck + four `0.02` r cylinder posts,
~20 kg, dynamic.

**2.1** Author `isaac/assets/cart.usda`:
- `/cart` — Xform, default prim
- `/cart/deck` — `Cube` `0.8 × 0.5 × 0.1` at `z = 0.5`
- `/cart/post_fl … post_br` — `Cylinder` `radius 0.02, height 0.5` at the corners
- on `/cart`: `UsdPhysics.RigidBodyAPI` + `UsdPhysics.MassAPI` with `mass = 20`
- on the geoms: `UsdPhysics.CollisionAPI` (or one box collider approximating the
  legs, mirroring the SDF)
- a `UsdShade.Material` (`OmniPBR`, muted blue) bound to `/cart`

**2.2 Verify:** reference `cart.usda` into an empty stage at `z = 1`, Play → it
falls, lands flat, doesn't jitter or explode. Tunnelling through the floor →
raise collision resolution or lower the physics timestep.

**Docs:** Isaac Sim → *Adding Materials*, *Rigid Bodies*.

---

## Phase 3 — Procedural layout (the direct reuse)

`layout_generator.py` is pure stdlib — import it unchanged.

**3.1** Make it importable from Isaac's Python. Quick route: at the top of your
script, `sys.path.insert(0, "<repo>/src/scene_spawner")` then
`from scene_spawner.layout_generator import LayoutParams, generate_layout`.
Cleaner: add a `pyproject.toml` to `src/scene_spawner` exposing just
`scene_spawner.layout_generator` and `pip install -e` it into the venv.

**3.2** Write `isaac/scripts/spawn_layout.py` (after `SimulationApp` is created and
`warehouse.usda` is loaded as the base stage):
```python
import math, sys, yaml
sys.path.insert(0, "<repo>/src/scene_spawner")
from scene_spawner.layout_generator import LayoutParams, generate_layout
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom
import omni.usd

stage = omni.usd.get_context().get_stage()
cfg = yaml.safe_load(open("<repo>/config/layout.yaml"))["/scene_spawner"]["ros__parameters"]
params = LayoutParams(
    count=cfg["default_count"],
    area=tuple(cfg["default_area"]),
    min_spacing=cfg["default_min_spacing"],
    footprint_radius=cfg["cart_footprint_radius"],
    seed=cfg["startup_seed"],
)
for p in generate_layout(params, name_prefix="cart"):
    prim_path = f"/World/carts/{p.name}"
    add_reference_to_stage("<repo>/isaac/assets/cart.usda", prim_path)
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.AddTranslateOp().Set(Gf.Vec3d(p.x, p.y, 0.0))
    xf.AddRotateZOp().Set(math.degrees(p.yaw))
```

**3.3 Verify the seam:** run `generate_layout` with `seed=0` in plain Python and in
your Isaac script — the list of `(x, y, yaw)` must be **identical**, and identical
to what Gazebo's `scene_spawner` places. Screenshot Gazebo and Isaac side by side.

**3.4** (optional) Re-implement `/scene_spawner/spawn_layout` as an Isaac extension
that edits the stage on a ROS 2 service call, reusing `SpawnLayout.srv`. Nice API
parity; a startup script is enough for a first port.

---

## Phase 4 — The robot (URDF → USD)

Isaac can't read SDF. Recreate `models/diffbot/model.sdf` as URDF, import it, then
add sensor + control.

**4.1** Write `isaac/assets/diffbot/diffbot.urdf`:
- `base_link` — box `0.4 × 0.3 × 0.15`, mass 6
- `left_wheel`, `right_wheel` — cylinders `r 0.11, len 0.05`, mass 0.5,
  `continuous` joints about the wheel axis; `wheel_separation 0.36`
- `caster` — small sphere link on a fixed joint (or a frictionless sphere collider
  on `base_link`, as the SDF now does)
- `lidar_link` — fixed to `base_link` at the front, as the sensor mount

**4.2 Import.** GUI: *Isaac Utils → Workflows → URDF Importer* — *Fixed Base = off*,
*Merge Fixed Joints*, *Self Collision = off* — save `diffbot.usd`. Or scripted with
`isaacsim.asset.importer.urdf`. **Verify:** the prim tree shows an `Articulation`
root on `base_link`; wheels are child links with revolute joints.

**4.3 Add the lidar:**
- **RTX Lidar** (`isaacsim.sensors.rtx`) — GPU ray-traced, realistic, JSON config
  profiles. *Create → Isaac → Sensors → RTX Lidar*, parent to `lidar_link`, config
  ~360 horizontal samples, 12 m range, 2π FOV to match the SDF `gpu_lidar`.
- **PhysX Lidar** (`isaacsim.sensors.physics`) — simpler, cheaper, fine for a demo.

**4.4 Add differential drive.** `isaacsim.robot.wheeled_robots` `WheeledRobot` +
`DifferentialController(wheel_radius=0.11, wheel_base=0.36)` in a script, or the
OmniGraph route in Phase 5. **Verify** (`isaac/scripts/drive_test.py`):
`DifferentialController.forward([0.3, 0.0])` each step → drives straight;
`[0.0, 0.5]` → spins in place.

**Docs:** Isaac Sim → *Import URDF*, *RTX Lidar*, *Wheeled Robots*.

---

## Phase 5 — ROS 2 bridge

Isaac's ROS 2 bridge is an **OmniGraph action graph**, not a config file.

**5.1** `source /opt/ros/humble/setup.bash` **before** launching Isaac Sim. Set
`ROS_DOMAIN_ID` to match your ROS terminals. Enable `isaacsim.ros2.bridge` in
*Window → Extensions* (or `--enable` on a standalone run).

**5.2** Build one action graph (*Window → Visual Scripting → Action Graph*, or
scripted with `omni.graph.core`). Match the topic names `bridge.yaml` used:

| OmniGraph nodes | ROS 2 result |
|---|---|
| `On Playback Tick` | drives the graph |
| `ROS2 Context` + `ROS2 Publish Clock` | `/clock` |
| `ROS2 Subscribe Twist` → `Differential Controller` → `Articulation Controller` | `/cmd_vel` drives the wheels |
| RTX Lidar / `Isaac Read Lidar Beams` → `ROS2 Publish Laser Scan` | `/scan` (`sensor_msgs/LaserScan`) |
| `Isaac Compute Odometry` → `ROS2 Publish Odometry` | `/odom` |
| `ROS2 Publish Transform Tree` (base_link + wheels) | `/tf` |

**5.3 Verify:** Play, then in a ROS terminal — `ros2 topic list` shows all five;
`ros2 topic echo /scan --once` returns a populated scan;
`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"` drives
the robot.

**Docs:** Isaac Sim → *ROS 2 → Driving TurtleBot* and *Publishing Sensor Data*
tutorials — close to exactly this.

---

## Phase 6 — Reuse `safety_stop` unchanged

`safety_stop` is pure ROS 2: subscribes `/cmd_vel_raw` + `/scan`, publishes
`/cmd_vel`. Isaac now provides `/scan` and consumes `/cmd_vel` — it drops in with
**no code change**.

**6.1** From the built `amr-sim-lab` workspace: `ros2 run safety_stop safety_stop_node`.
**6.2** `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw`.
**6.3 Verify — the payoff:** drive straight at a cart. The robot halts ~0.6 m
short, still rotates/reverses. **Byte-identical behaviour to Gazebo, same node.**

---

## Phase 7 — Replicator / synthetic data *(optional, high value)*

Covers the "synthetic data generation pipelines" gap directly.

**7.1** `isaac/scripts/replicator_carts.py`:
```python
import omni.replicator.core as rep
# ... load warehouse, set up a camera ...

carts = [rep.create.from_usd("<repo>/isaac/assets/cart.usda") for _ in range(8)]
lights = rep.create.light(light_type="distant")
for c in carts:
    rep.modify.semantics([("class", "cart")], c)

with rep.trigger.on_frame(num_frames=300):
    with rep.get.prims(carts):
        rep.modify.pose(position=...)          # from generate_layout(<per-frame seed>)
        rep.randomizer.materials(...)          # colour / roughness jitter
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
        rep.modify.pose(rotation=rep.distribution.uniform((-30, -30, 0), (30, 30, 0)))

writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="isaac/out/sdg",
                  rgb=True, bounding_box_2d_tight=True, semantic_segmentation=True)
writer.attach([rep.create.render_product(camera, (1280, 720))])
rep.orchestrator.run()
```
Drive the per-frame cart poses from `generate_layout` with a fresh seed each frame,
so the synthetic set uses the *same* placement logic as the sim.

**7.2** Run headless: `python isaac/scripts/replicator_carts.py`.
**7.3 Verify:** `isaac/out/sdg/` has ~300 RGB PNGs + per-frame JSON with 2D boxes
and segmentation masks. Open a few — boxes line up on the carts.

**Docs:** *Replicator → Getting Started*, *Annotators & Writers*, *Randomizers*.

---

## Phase 8 — Isaac Lab RL *(optional stretch — timebox hard)*

A working scene + Replicator + README beats a half-trained policy.

**8.1** `git clone https://github.com/isaac-sim/IsaacLab && cd IsaacLab && ./isaaclab.sh --install`.
**8.2** Run a **stock** task to learn the loop:
`./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Cartpole-v0 --headless`
— watch it vectorize thousands of envs on one GPU.
**8.3** Write up (for `docs/gazebo-vs-isaac.md`): how Isaac Lab tensorizes N
environments and batches the physics step vs. one Gazebo world at RTF 1.0 — that
is the "GPU-scale simulation architecture" talking point, and you get it from
*watching* the cartpole run.
**8.4** (only if time) wrap the diffbot as a `DirectRLEnv` with a
reach-a-goal-avoiding-carts reward. Real work — do not start before the
application is out.

---

## Phase 9 — Document

**9.1** Fill in `docs/gazebo-vs-isaac.md` (currently a stub) with what each phase
taught you: SDF `<include>` vs USD references/payloads; DART CPU fixed-step vs
PhysX GPU; `gz-sim` systems vs OmniGraph nodes; `ros_gz_bridge` YAML vs the ROS 2
action graph; `gz service` spawn vs stage prim manipulation.

**9.2** `isaac/README.md`: the seam story (what ported unchanged), a
Gazebo-vs-Isaac side-by-side of the same seed-0 layout, and a Replicator output
sample.

---

## Reference links

- Isaac Sim docs — <https://docs.isaacsim.omniverse.nvidia.com/>
- Learn OpenUSD (browser, no GPU) — <https://docs.nvidia.com/learn-openusd/latest/index.html>
- Isaac Sim ROS 2 tutorials — <https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/index.html>
- Replicator — <https://docs.omniverse.nvidia.com/py/replicator/>
- Isaac Lab — <https://isaac-sim.github.io/IsaacLab/>
- NVIDIA Robotics learning path — <https://www.nvidia.com/en-us/learn/learning-path/robotics/>
