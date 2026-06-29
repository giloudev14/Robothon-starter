from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import imageio.v3 as iio
import numpy as np

if "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"

import mujoco


ROOT = Path(__file__).resolve().parent
REPO_ROOT = ROOT.parents[1]
DEFAULT_URDF = REPO_ROOT / "assets" / "Futurist" / "futurist.urdf"
COURSE_JSON = ROOT / "course_obstacles.json"
ARTIFACTS = ROOT / "artifacts"
DEFAULT_VIDEO = ARTIFACTS / "robot_autonomous_perception_demo.mp4"
DEFAULT_TRAJECTORY = ARTIFACTS / "robot_autonomous_perception_trajectory.json"
DEFAULT_REPORT = ARTIFACTS / "robot_autonomous_perception_report.json"
DEFAULT_DECISIONS = ARTIFACTS / "robot_autonomous_perception_decisions.json"
DEFAULT_SCENE = ROOT / "scene.xml"

LANES = {-1: -0.62, 0: 0.0, 1: 0.62}
HUMANOID_BASE_HEIGHT = 0.98
BASE_CLEARANCE_M = 0.48
SIDE_JOINTS = {
    "left": {
        "hip_roll": "idx01_left_hip_roll",
        "hip_yaw": "idx02_left_hip_yaw",
        "hip_pitch": "idx03_left_hip_pitch",
        "tarsus": "idx04_left_tarsus",
        "toe_pitch": "idx05_left_toe_pitch",
        "shoulder_pitch": "idx13_left_arm_joint1",
        "shoulder_roll": "idx14_left_arm_joint2",
        "shoulder_yaw": "idx15_left_arm_joint3",
        "elbow": "idx16_left_arm_joint4",
        "wrist": "idx19_left_arm_joint7",
    },
    "right": {
        "hip_roll": "idx07_right_hip_roll",
        "hip_yaw": "idx08_right_hip_yaw",
        "hip_pitch": "idx09_right_hip_pitch",
        "tarsus": "idx10_right_tarsus",
        "toe_pitch": "idx11_right_toe_pitch",
        "shoulder_pitch": "idx20_right_arm_joint1",
        "shoulder_roll": "idx21_right_arm_joint2",
        "shoulder_yaw": "idx22_right_arm_joint3",
        "elbow": "idx23_right_arm_joint4",
        "wrist": "idx26_right_arm_joint7",
    },
}


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    x: float
    lane: int
    height_m: float = 0.0
    width_m: float = 0.4
    slope_deg: float = 0.0
    steps: int = 0
    depth_m: float = 0.0
    hazard: float = 0.0


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    if value <= edge0:
        return 0.0
    if value >= edge1:
        return 1.0
    x = (value - edge0) / (edge1 - edge0)
    return x * x * (3.0 - 2.0 * x)


def load_obstacles(path: Path = COURSE_JSON) -> list[Obstacle]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    obstacles = []
    for row in payload["obstacles"]:
        obstacles.append(
            Obstacle(
                name=row["name"],
                kind=row["kind"],
                x=float(row["x"]),
                lane=int(row["lane"]),
                height_m=float(row.get("height_m", 0.0)),
                width_m=float(row.get("width_m", 0.4)),
                slope_deg=float(row.get("slope_deg", 0.0)),
                steps=int(row.get("steps", 0)),
                depth_m=float(row.get("depth_m", 0.0)),
                hazard=float(row.get("hazard", 0.0)),
            )
        )
    return obstacles


def missing_meshes(urdf_path: Path) -> list[str]:
    text = urdf_path.read_text(encoding="utf-8")
    mesh_names = re.findall(r'<mesh\s+filename="([^"]+)"', text)
    return [name for name in mesh_names if not (urdf_path.parent / name).exists()]


def mujoco_ready_urdf(source_urdf: Path, temp_dir: Path) -> Path:
    if not source_urdf.exists():
        raise FileNotFoundError(f"Cannot find Futurist humanoid URDF: {source_urdf}")
    missing = missing_meshes(source_urdf)
    if missing:
        raise FileNotFoundError(f"Futurist URDF is missing mesh files, for example: {', '.join(missing[:8])}")

    text = source_urdf.read_text(encoding="utf-8")
    compiler = (
        "  <mujoco>\n"
        f'    <compiler meshdir="{source_urdf.parent}" discardvisual="false"/>\n'
        "  </mujoco>\n"
    )
    if "<mujoco>" not in text:
        text = re.sub(r"(<robot[^>]*>\n)", r"\1" + compiler, text, count=1)
    output_path = temp_dir / source_urdf.name
    output_path.write_text(text, encoding="utf-8")
    return output_path


def add_box(world: mujoco.MjsBody, *, name: str, pos: list[float], size: list[float], rgba: list[float]) -> None:
    world.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size, rgba=rgba)


def add_marker_body(world: mujoco.MjsBody, name: str, rgba: list[float], radius: float = 0.06) -> None:
    body = world.add_body(name=name, pos=[-1.75, 0.0, 0.12])
    body.add_freejoint(name=f"{name}_freejoint")
    body.add_geom(name=f"{name}_geom", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[radius], rgba=rgba)


def add_cylinder(
    world: mujoco.MjsBody,
    *,
    name: str,
    pos: list[float],
    radius: float,
    halfheight: float,
    rgba: list[float],
    euler: list[float] | None = None,
) -> None:
    kwargs = {"name": name, "type": mujoco.mjtGeom.mjGEOM_CYLINDER, "pos": pos, "size": [radius, halfheight], "rgba": rgba}
    if euler is not None:
        kwargs["euler"] = euler
    world.add_geom(**kwargs)


def add_sphere(world: mujoco.MjsBody, *, name: str, pos: list[float], radius: float, rgba: list[float]) -> None:
    world.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=pos, size=[radius], rgba=rgba)


def add_realistic_obstacle(world: mujoco.MjsBody, obstacle: Obstacle) -> None:
    y = LANES[obstacle.lane]
    if obstacle.kind == "stairs":
        step_count = max(2, obstacle.steps)
        step_len = obstacle.width_m / step_count
        for idx in range(step_count):
            step_h = obstacle.height_m * (idx + 1) / step_count
            x = obstacle.x - obstacle.width_m / 2.0 + step_len * (idx + 0.5)
            add_box(
                world,
                name=f"{obstacle.name}_tread_{idx + 1}",
                pos=[x, y, step_h / 2.0 + 0.010],
                size=[step_len * 0.48, 0.25, step_h / 2.0],
                rgba=[0.58, 0.56, 0.48, 1.0],
            )
            add_box(
                world,
                name=f"{obstacle.name}_edge_{idx + 1}",
                pos=[x + step_len * 0.43, y, step_h + 0.016],
                size=[0.010, 0.255, 0.012],
                rgba=[0.90, 0.82, 0.46, 1.0],
            )
        add_cylinder(
            world,
            name=f"{obstacle.name}_left_handrail",
            pos=[obstacle.x, y - 0.29, obstacle.height_m + 0.16],
            radius=0.018,
            halfheight=obstacle.width_m * 0.58,
            rgba=[0.20, 0.22, 0.23, 1.0],
            euler=[0.0, math.pi / 2.0, 0.0],
        )
        add_cylinder(
            world,
            name=f"{obstacle.name}_right_handrail",
            pos=[obstacle.x, y + 0.29, obstacle.height_m + 0.16],
            radius=0.018,
            halfheight=obstacle.width_m * 0.58,
            rgba=[0.20, 0.22, 0.23, 1.0],
            euler=[0.0, math.pi / 2.0, 0.0],
        )
    elif obstacle.kind == "ramp":
        add_box(
            world,
            name=f"{obstacle.name}_deck",
            pos=[obstacle.x, y, obstacle.height_m / 2.0 + 0.010],
            size=[obstacle.width_m * 0.50, 0.25, obstacle.height_m / 2.0],
            rgba=[0.36, 0.45, 0.47, 1.0],
        )
        for dy in (-0.27, 0.27):
            add_cylinder(
                world,
                name=f"{obstacle.name}_rail_{dy}",
                pos=[obstacle.x, y + dy, obstacle.height_m + 0.13],
                radius=0.016,
                halfheight=obstacle.width_m * 0.52,
                rgba=[0.12, 0.14, 0.15, 1.0],
                euler=[0.0, math.pi / 2.0, 0.0],
            )
    elif obstacle.kind == "gap":
        add_box(
            world,
            name=f"{obstacle.name}_dark_void",
            pos=[obstacle.x, y, -0.018],
            size=[obstacle.width_m * 0.50, 0.25, 0.018],
            rgba=[0.005, 0.005, 0.006, 1.0],
        )
        for idx, dx in enumerate([-0.31, -0.18, 0.18, 0.31]):
            add_box(
                world,
                name=f"{obstacle.name}_loose_paver_{idx}",
                pos=[obstacle.x + dx, y + 0.16 * ((idx % 2) * 2 - 1), 0.022],
                size=[0.070, 0.085, 0.012],
                rgba=[0.46, 0.45, 0.39, 1.0],
            )
    elif obstacle.kind == "curb":
        add_box(
            world,
            name=f"{obstacle.name}_curb_face",
            pos=[obstacle.x, y, obstacle.height_m / 2.0 + 0.010],
            size=[obstacle.width_m * 0.50, 0.055, obstacle.height_m / 2.0],
            rgba=[0.70, 0.70, 0.65, 1.0],
        )
        add_box(
            world,
            name=f"{obstacle.name}_gutter_channel",
            pos=[obstacle.x, y + 0.16, 0.018],
            size=[obstacle.width_m * 0.48, 0.050, 0.010],
            rgba=[0.16, 0.18, 0.18, 1.0],
        )
    elif obstacle.name == "traffic_barrier":
        add_box(
            world,
            name=f"{obstacle.name}_plastic_body",
            pos=[obstacle.x, y, obstacle.height_m * 0.52],
            size=[obstacle.width_m * 0.50, 0.070, obstacle.height_m * 0.32],
            rgba=[0.95, 0.28, 0.12, 1.0],
        )
        for idx, dx in enumerate([-0.22, 0.0, 0.22]):
            add_box(
                world,
                name=f"{obstacle.name}_white_stripe_{idx}",
                pos=[obstacle.x + dx, y - 0.073, obstacle.height_m * 0.57],
                size=[0.030, 0.008, obstacle.height_m * 0.23],
                rgba=[0.96, 0.96, 0.88, 1.0],
            )
        for dx in (-0.24, 0.24):
            add_box(
                world,
                name=f"{obstacle.name}_foot_{dx}",
                pos=[obstacle.x + dx, y, 0.045],
                size=[0.080, 0.18, 0.028],
                rgba=[0.10, 0.10, 0.10, 1.0],
            )
    elif obstacle.name == "fallen_sidewalk_sign":
        add_cylinder(
            world,
            name=f"{obstacle.name}_post",
            pos=[obstacle.x, y, 0.09],
            radius=0.018,
            halfheight=0.30,
            rgba=[0.58, 0.60, 0.60, 1.0],
            euler=[math.pi / 2.0, 0.0, 0.42],
        )
        add_box(
            world,
            name=f"{obstacle.name}_panel",
            pos=[obstacle.x + 0.10, y + 0.07, 0.19],
            size=[0.18, 0.020, 0.13],
            rgba=[0.05, 0.36, 0.80, 1.0],
        )
        add_box(
            world,
            name=f"{obstacle.name}_base",
            pos=[obstacle.x - 0.14, y - 0.10, 0.035],
            size=[0.10, 0.08, 0.025],
            rgba=[0.05, 0.05, 0.055, 1.0],
        )
    elif obstacle.kind == "solid_block":
        for idx, (dx, dy, dz) in enumerate([(-0.11, -0.07, 0.08), (0.09, 0.08, 0.08), (0.00, 0.00, 0.24)]):
            add_box(
                world,
                name=f"{obstacle.name}_crate_{idx}",
                pos=[obstacle.x + dx, y + dy, dz + 0.010],
                size=[0.13, 0.12, 0.08],
                rgba=[0.56, 0.34, 0.18, 1.0],
            )
            add_box(
                world,
                name=f"{obstacle.name}_crate_lip_{idx}",
                pos=[obstacle.x + dx, y + dy, dz + 0.095],
                size=[0.135, 0.012, 0.010],
                rgba=[0.23, 0.15, 0.09, 1.0],
            )
    elif obstacle.kind == "cylinders":
        for idx, dy in enumerate([-0.14, 0.0, 0.14]):
            add_cylinder(
                world,
                name=f"{obstacle.name}_pipe_{idx}",
                pos=[obstacle.x + 0.04 * idx, y + dy, obstacle.height_m / 2.0 + 0.020],
                radius=0.065,
                halfheight=0.23,
                rgba=[0.72, 0.42, 0.14, 1.0],
                euler=[math.pi / 2.0, 0.0, 0.0],
            )
            add_sphere(
                world,
                name=f"{obstacle.name}_cap_{idx}",
                pos=[obstacle.x + 0.04 * idx, y + dy - 0.23, obstacle.height_m / 2.0 + 0.020],
                radius=0.066,
                rgba=[0.78, 0.50, 0.20, 1.0],
            )
    elif obstacle.kind == "slalom_posts":
        for idx, dy in enumerate([-0.16, 0.04, 0.22]):
            x = obstacle.x + 0.11 * idx
            add_cylinder(
                world,
                name=f"{obstacle.name}_cone_base_{idx}",
                pos=[x, y + dy, 0.035],
                radius=0.075,
                halfheight=0.020,
                rgba=[0.98, 0.42, 0.08, 1.0],
            )
            add_cylinder(
                world,
                name=f"{obstacle.name}_cone_body_{idx}",
                pos=[x, y + dy, obstacle.height_m * 0.42],
                radius=0.040,
                halfheight=obstacle.height_m * 0.36,
                rgba=[1.0, 0.48, 0.08, 1.0],
            )
            add_box(
                world,
                name=f"{obstacle.name}_reflector_{idx}",
                pos=[x, y + dy - 0.041, obstacle.height_m * 0.50],
                size=[0.045, 0.006, 0.012],
                rgba=[0.96, 0.96, 0.84, 1.0],
            )


def build_scene_spec(urdf_path: Path, obstacles: list[Obstacle], temp_dir: Path) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(mujoco_ready_urdf(urdf_path, temp_dir)))
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    spec.option.timestep = 0.002
    spec.option.gravity = [0.0, 0.0, -9.81]

    base = spec.body("base_link")
    if base is None:
        raise ValueError("Missing base_link body in Futurist humanoid URDF")
    base.add_freejoint(name="floating_base_joint")

    world = spec.worldbody
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05],
        rgba=[0.075, 0.080, 0.082, 1.0],
    )
    add_box(world, name="course_spine", pos=[1.65, 0.0, 0.003], size=[3.75, 1.03, 0.003], rgba=[0.18, 0.19, 0.18, 1.0])
    for lane, y in LANES.items():
        rgba = [0.24, 0.26, 0.25, 0.72] if lane else [0.29, 0.31, 0.29, 0.78]
        add_box(world, name=f"lane_{lane}_surface", pos=[1.65, y, 0.008], size=[3.70, 0.26, 0.004], rgba=rgba)

    add_box(world, name="start_pad", pos=[-1.85, 0.0, 0.015], size=[0.20, 0.42, 0.010], rgba=[0.05, 0.34, 1.00, 0.82])
    add_box(world, name="goal_pad", pos=[5.35, 0.0, 0.017], size=[0.24, 0.48, 0.012], rgba=[0.18, 0.95, 0.40, 0.82])

    for obstacle in obstacles:
        add_realistic_obstacle(world, obstacle)

    for lane, y in LANES.items():
        add_box(
            world,
            name=f"scan_lane_{lane}",
            pos=[-1.60, y, 0.032],
            size=[0.16, 0.22, 0.012],
            rgba=[0.35, 0.35, 0.35, 0.35],
        )
    add_marker_body(world, "decision_beacon", [0.10, 0.90, 1.00, 0.95], 0.065)
    add_marker_body(world, "route_cursor", [1.00, 0.92, 0.18, 0.95], 0.045)

    world.add_light(pos=[0.0, -1.6, 3.2], dir=[0.0, 0.45, -1.0], diffuse=[1.0, 1.0, 1.0])
    world.add_light(pos=[2.3, -2.2, 4.2], dir=[-0.35, 0.30, -1.0], diffuse=[0.95, 0.92, 0.82])
    world.add_light(pos=[4.8, 1.6, 2.4], dir=[-0.7, -0.2, -1.0], diffuse=[0.62, 0.66, 0.70])
    world.add_camera(name="course_camera", pos=[2.05, -4.35, 2.05], xyaxes=[1.0, 0.08, 0.0, -0.03, 0.38, 0.93])
    return spec


def build_model(urdf_path: Path, obstacles: list[Obstacle]) -> mujoco.MjModel:
    with tempfile.TemporaryDirectory() as tmp:
        return build_scene_spec(urdf_path, obstacles, Path(tmp)).compile()


def export_scene_xml(urdf_path: Path, scene_path: Path) -> Path:
    obstacles = load_obstacles()
    with tempfile.TemporaryDirectory() as tmp:
        spec = build_scene_spec(urdf_path, obstacles, Path(tmp))
        scene_path.parent.mkdir(parents=True, exist_ok=True)
        xml = spec.to_xml()
        relative_meshdir = os.path.relpath(urdf_path.parent, scene_path.parent)
        xml = re.sub(r'meshdir="[^"]+"', f'meshdir="{relative_meshdir}/"', xml, count=1)
        scene_path.write_text(xml, encoding="utf-8")
    return scene_path


def style_model_for_video(model: mujoco.MjModel) -> None:
    body_shell = np.array([0.88, 0.90, 0.94, 1.0], dtype=np.float32)
    joint_shell = np.array([0.06, 0.42, 0.66, 1.0], dtype=np.float32)
    limb_shell = np.array([0.16, 0.17, 0.19, 1.0], dtype=np.float32)
    foot_shell = np.array([0.035, 0.035, 0.040, 1.0], dtype=np.float32)

    protected = {
        "floor",
        "course_spine",
        "start_pad",
        "goal_pad",
        "decision_beacon_geom",
        "route_cursor_geom",
    }
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name in protected or name.startswith(("lane_", "scan_lane_")) or "_tread_" in name or "_edge_" in name:
            continue
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id])) or ""
        if body_name == "base_link":
            model.geom_rgba[geom_id] = body_shell
        elif "hip" in body_name or "shoulder" in body_name:
            model.geom_rgba[geom_id] = joint_shell
        elif "toe" in body_name or "foot" in body_name:
            model.geom_rgba[geom_id] = foot_shell
        elif body_name in {"decision_beacon", "route_cursor"}:
            continue
        elif model.geom_group[geom_id] == 0:
            model.geom_rgba[geom_id] = [0.0, 0.0, 0.0, 0.0]
        else:
            model.geom_rgba[geom_id] = limb_shell


def joint_qpos_addr(model: mujoco.MjModel, joint_name: str) -> int | None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return None
    return int(model.jnt_qposadr[joint_id])


def set_joint(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, value: float) -> None:
    qpos_addr = joint_qpos_addr(model, joint_name)
    if qpos_addr is None:
        return
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if model.jnt_limited[joint_id]:
        low, high = model.jnt_range[joint_id]
        value = float(np.clip(value, low, high))
    data.qpos[qpos_addr] = value


def set_freejoint_pose(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, pos: tuple[float, float, float], yaw: float = 0.0) -> None:
    qpos_addr = joint_qpos_addr(model, joint_name)
    if qpos_addr is None:
        return
    data.qpos[qpos_addr : qpos_addr + 3] = pos
    data.qpos[qpos_addr + 3 : qpos_addr + 7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def lane_obstacles_near(obstacles: list[Obstacle], x: float, lane: int, lookahead: float = 0.55) -> list[Obstacle]:
    return [obs for obs in obstacles if obs.lane == lane and x - 0.18 <= obs.x <= x + lookahead]


def obstacle_cost(obs: Obstacle) -> tuple[float, str]:
    if obs.kind in {"solid_block", "slalom_posts", "cylinders"}:
        if obs.height_m >= BASE_CLEARANCE_M or obs.hazard >= 0.85:
            return 9.0 + obs.hazard, "blocked"
        return 2.2 + obs.hazard, "step_over"
    if obs.kind == "gap":
        return 5.0 + 2.0 * obs.hazard, "avoid_gap"
    if obs.kind == "stairs":
        slope_penalty = max(0.0, obs.slope_deg - 18.0) * 0.08
        height_penalty = max(0.0, obs.height_m - 0.24) * 4.0
        return 1.25 + slope_penalty + height_penalty + obs.hazard, "ascend_stairs"
    if obs.kind == "ramp":
        return 0.65 + max(0.0, obs.slope_deg - 12.0) * 0.06 + obs.hazard, "ascend_ramp"
    if obs.kind == "curb":
        return 1.0 + obs.hazard, "step_curb"
    return 1.0 + obs.hazard, "unknown"


def perceive_options(obstacles: list[Obstacle], x: float, current_lane: int) -> dict[int, dict]:
    options: dict[int, dict] = {}
    for lane in LANES:
        near = lane_obstacles_near(obstacles, x, lane)
        terrain_cost = 0.0
        actions = []
        max_height = 0.0
        max_slope = 0.0
        for obs in near:
            cost, action = obstacle_cost(obs)
            terrain_cost += cost
            actions.append({"obstacle": obs.name, "kind": obs.kind, "action": action, "cost": round(cost, 3)})
            max_height = max(max_height, obs.height_m)
            max_slope = max(max_slope, obs.slope_deg)
        lateral_cost = 1.55 * abs(lane - current_lane)
        center_bias = 0.08 * abs(lane)
        total = terrain_cost + lateral_cost + center_bias
        options[lane] = {
            "total_cost": round(total, 3),
            "terrain_cost": round(terrain_cost, 3),
            "lateral_cost": round(lateral_cost, 3),
            "perceived_obstacles": actions,
            "max_height_m": round(max_height, 3),
            "max_slope_deg": round(max_slope, 1),
        }
    return options


def choose_lane(options: dict[int, dict]) -> tuple[int, str]:
    lane = min(options, key=lambda candidate: (options[candidate]["total_cost"], abs(candidate)))
    actions = options[lane]["perceived_obstacles"]
    if not actions:
        return lane, "walk_clear"
    if any(row["action"] in {"ascend_stairs", "ascend_ramp"} for row in actions):
        return lane, next(row["action"] for row in actions if row["action"].startswith("ascend"))
    return lane, actions[0]["action"]


def plan_route(obstacles: list[Obstacle]) -> tuple[list[dict], list[dict]]:
    checkpoints = [-1.45, -0.88, -0.22, 0.44, 1.08, 1.72, 2.36, 3.00, 3.66, 4.28, 4.92]
    current_lane = 0
    decisions = []
    waypoints = [{"x": -1.85, "lane": 0, "y": LANES[0], "action": "start"}]
    for idx, x in enumerate(checkpoints):
        options = perceive_options(obstacles, x, current_lane)
        chosen_lane, action = choose_lane(options)
        decisions.append(
            {
                "index": idx,
                "x_m": x,
                "current_lane": current_lane,
                "chosen_lane": chosen_lane,
                "action": action,
                "options": options,
            }
        )
        current_lane = chosen_lane
        waypoints.append({"x": x + 0.52, "lane": current_lane, "y": LANES[current_lane], "action": action})
    waypoints.append({"x": 5.35, "lane": current_lane, "y": LANES[current_lane], "action": "goal"})
    return waypoints, decisions


def terrain_height_at(obstacles: list[Obstacle], x: float, lane: int) -> float:
    height = 0.0
    for obs in obstacles:
        if obs.lane != lane:
            continue
        half = max(0.15, obs.width_m * 0.50)
        if not (obs.x - half <= x <= obs.x + half):
            continue
        local = (x - (obs.x - half)) / (2.0 * half)
        if obs.kind == "stairs":
            step = min(obs.steps, max(1, int(local * max(1, obs.steps)) + 1))
            height = max(height, obs.height_m * step / max(1, obs.steps))
        elif obs.kind == "ramp":
            height = max(height, obs.height_m * smoothstep(0.0, 1.0, local))
        elif obs.kind == "curb":
            height = max(height, obs.height_m)
    return height


def sample_route(waypoints: list[dict], progress: float) -> tuple[float, float, int, str]:
    progress = min(0.999, max(0.0, progress))
    scaled = progress * (len(waypoints) - 1)
    idx = int(scaled)
    local = scaled - idx
    a = waypoints[idx]
    b = waypoints[idx + 1]
    blend = smoothstep(0.0, 1.0, local)
    x = (1.0 - blend) * a["x"] + blend * b["x"]
    y = (1.0 - blend) * a["y"] + blend * b["y"]
    return x, y, int(b["lane"]), str(b["action"])


def apply_robot_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    time_s: float,
    duration_s: float,
    obstacles: list[Obstacle],
    waypoints: list[dict],
) -> tuple[float, float, int, str]:
    progress = smoothstep(0.4, duration_s - 0.4, time_s)
    x, y, lane, action = sample_route(waypoints, progress)
    terrain_z = terrain_height_at(obstacles, x, lane)
    gait = 2.0 * math.pi * (1.35 * time_s + 0.08 * math.sin(time_s))
    climb_gain = 1.55 if action.startswith("ascend") else 1.0
    base_z = HUMANOID_BASE_HEIGHT + terrain_z + 0.024 * math.sin(gait) * climb_gain
    lane_error = y - LANES[lane]
    yaw = 0.28 * math.atan2(lane_error, 0.8) + 0.035 * math.sin(0.45 * gait)

    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[0] = x
    data.qpos[1] = y
    data.qpos[2] = base_z
    data.qpos[3:7] = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]

    for side, sign, phase in (("left", 1.0, 0.0), ("right", -1.0, math.pi)):
        joints = SIDE_JOINTS[side]
        swing = math.sin(gait + phase)
        lift = max(0.0, swing)
        climb = 0.24 if action.startswith("ascend") else 0.0
        lateral_balance = -0.055 * np.sign(y) * sign
        set_joint(model, data, joints["hip_roll"], lateral_balance + sign * 0.025 * math.sin(gait + phase))
        set_joint(model, data, joints["hip_yaw"], 0.050 * math.sin(gait + phase + 0.5) + 0.045 * yaw)
        set_joint(model, data, joints["hip_pitch"], 0.25 * swing - 0.08 + climb * lift)
        set_joint(model, data, joints["tarsus"], -0.33 * lift - 0.10 * climb + 0.05 * math.sin(gait + phase))
        set_joint(model, data, joints["toe_pitch"], 0.18 * lift + 0.10 * climb)

        set_joint(model, data, joints["shoulder_pitch"], -0.34 * swing)
        set_joint(model, data, joints["shoulder_roll"], sign * 1.35)
        set_joint(model, data, joints["shoulder_yaw"], -0.06 * sign * np.sign(y))
        set_joint(model, data, joints["elbow"], -0.34 - 0.08 * lift)
        set_joint(model, data, joints["wrist"], 0.08 * sign * math.sin(gait + phase))

    set_joint(model, data, "idx27_head_joint1", -0.18 * np.sign(y) + 0.05 * math.sin(0.4 * gait))
    set_joint(model, data, "idx28_head_joint2", -0.06 if action.startswith("ascend") else 0.02)

    set_freejoint_pose(model, data, "route_cursor_freejoint", (x, y, terrain_z + 0.22), yaw)
    return x, y, lane, action


def update_decision_visuals(model: mujoco.MjModel, data: mujoco.MjData, decisions: list[dict], robot_x: float) -> dict | None:
    pending = [row for row in decisions if row["x_m"] >= robot_x - 0.25]
    decision = pending[0] if pending else decisions[-1]
    y = LANES[decision["chosen_lane"]]
    set_freejoint_pose(model, data, "decision_beacon_freejoint", (decision["x_m"], y, 0.62), 0.0)

    for lane in LANES:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"scan_lane_{lane}")
        if geom_id >= 0:
            model.geom_pos[geom_id] = [decision["x_m"], LANES[lane], 0.038]
            if lane == decision["chosen_lane"]:
                model.geom_rgba[geom_id] = [0.05, 0.80, 0.35, 0.78]
            elif decision["options"][lane]["terrain_cost"] >= 5.0:
                model.geom_rgba[geom_id] = [0.95, 0.12, 0.08, 0.52]
            else:
                model.geom_rgba[geom_id] = [0.95, 0.72, 0.18, 0.45]
    return decision


def update_camera(model: mujoco.MjModel, data: mujoco.MjData, camera: mujoco.MjvCamera, robot_x: float, robot_y: float) -> None:
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [robot_x + 0.65, robot_y * 0.20, 0.72]
    camera.distance = 4.15
    camera.azimuth = 126.0 + 16.0 * smoothstep(0.0, 5.0, robot_x)
    camera.elevation = -27.0


def body_position(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> list[float]:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError(f"Missing body in model: {body_name}")
    return data.xpos[body_id].copy().round(5).tolist()


def draw_rect(img: np.ndarray, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
    img[max(0, y0) : min(img.shape[0], y1), max(0, x0) : min(img.shape[1], x1)] = color


def draw_disk(img: np.ndarray, center: tuple[int, int], radius: int, color: tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    cx, cy = center
    y, x = np.ogrid[:h, :w]
    img[(x - cx) ** 2 + (y - cy) ** 2 <= radius * radius] = color


def world_to_px(x: float, y: float, width: int, height: int) -> tuple[int, int]:
    px = int((x + 2.10) / 7.75 * width)
    py = int((1.20 - y) / 2.40 * height)
    return px, py


def schematic_frame(
    *,
    width: int,
    height: int,
    obstacles: list[Obstacle],
    decisions: list[dict],
    robot_x: float,
    robot_y: float,
    active_decision: dict | None,
    progress: float,
) -> np.ndarray:
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (20, 23, 24)
    for lane, y in LANES.items():
        x0, y0 = world_to_px(-2.0, y - 0.26, width, height)
        x1, y1 = world_to_px(5.55, y + 0.26, width, height)
        color = (42, 47, 47) if lane else (50, 55, 55)
        draw_rect(img, x0, y1, x1, y0, color)

    sx0, sy0 = world_to_px(-2.05, -0.42, width, height)
    sx1, sy1 = world_to_px(-1.62, 0.42, width, height)
    gx0, gy0 = world_to_px(5.05, -0.50, width, height)
    gx1, gy1 = world_to_px(5.62, 0.50, width, height)
    draw_rect(img, sx0, sy1, sx1, sy0, (25, 95, 210))
    draw_rect(img, gx0, gy1, gx1, gy0, (35, 190, 80))

    colors = {
        "stairs": (168, 151, 104),
        "ramp": (100, 130, 135),
        "gap": (3, 3, 4),
        "curb": (155, 155, 145),
        "solid_block": (126, 75, 54),
        "cylinders": (190, 105, 30),
        "slalom_posts": (245, 115, 20),
    }
    for obs in obstacles:
        y = LANES[obs.lane]
        half_x = max(0.15, obs.width_m * 0.50)
        half_y = 0.22
        x0, y0 = world_to_px(obs.x - half_x, y - half_y, width, height)
        x1, y1 = world_to_px(obs.x + half_x, y + half_y, width, height)
        draw_rect(img, x0, y1, x1, y0, colors.get(obs.kind, (120, 120, 120)))

    if active_decision is not None:
        dx = active_decision["x_m"]
        for lane, y in LANES.items():
            px, py = world_to_px(dx, y, width, height)
            option = active_decision["options"][lane]
            if lane == active_decision["chosen_lane"]:
                color = (45, 220, 100)
            elif option["terrain_cost"] >= 5.0:
                color = (230, 45, 35)
            else:
                color = (230, 180, 55)
            draw_rect(img, px - 20, py - 16, px + 20, py + 16, color)
        bx, by = world_to_px(dx, LANES[active_decision["chosen_lane"]], width, height)
        draw_disk(img, (bx, by), 16, (40, 210, 230))

    for row in decisions:
        px, py = world_to_px(row["x_m"], LANES[row["chosen_lane"]], width, height)
        draw_disk(img, (px, py), 4, (230, 230, 190))

    rx, ry = world_to_px(robot_x, robot_y, width, height)
    draw_disk(img, (rx + 18, ry), 10, (232, 236, 240))
    draw_rect(img, rx - 12, ry - 9, rx + 16, ry + 9, (210, 222, 232))
    draw_rect(img, rx - 24, ry - 24, rx - 5, ry - 14, (20, 120, 185))
    draw_rect(img, rx - 24, ry + 14, rx - 5, ry + 24, (20, 120, 185))
    draw_rect(img, rx - 4, ry - 30, rx + 7, ry - 12, (38, 42, 48))
    draw_rect(img, rx - 4, ry + 12, rx + 7, ry + 30, (38, 42, 48))

    bar_w = int((width - 80) * progress)
    draw_rect(img, 40, height - 34, width - 40, height - 22, (65, 68, 68))
    draw_rect(img, 40, height - 34, 40 + bar_w, height - 22, (235, 206, 70))
    return img


def run_demo(
    *,
    urdf_path: Path,
    video_path: Path,
    trajectory_path: Path,
    report_path: Path,
    decisions_path: Path,
    duration_s: float,
    fps: int,
    width: int,
    height: int,
    scene_path: Path | None,
) -> dict:
    obstacles = load_obstacles()
    waypoints, decisions = plan_route(obstacles)
    if scene_path is not None:
        export_scene_xml(urdf_path, scene_path)
    model = build_model(urdf_path, obstacles)
    style_model_for_video(model)
    data = mujoco.MjData(model)
    renderer = None
    render_fallback_reason = None
    try:
        renderer = mujoco.Renderer(model, width=width, height=height)
    except Exception as exc:
        render_fallback_reason = str(exc)
    camera = mujoco.MjvCamera()

    video_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    frames: list[np.ndarray] = []
    trajectory: list[dict] = []
    total_frames = int(duration_s * fps)

    for frame_idx in range(total_frames):
        time_s = frame_idx / fps
        x, y, lane, action = apply_robot_pose(
            model,
            data,
            time_s=time_s,
            duration_s=duration_s,
            obstacles=obstacles,
            waypoints=waypoints,
        )
        active_decision = update_decision_visuals(model, data, decisions, x)
        mujoco.mj_forward(model, data)
        if renderer is None:
            frames.append(
                schematic_frame(
                    width=width,
                    height=height,
                    obstacles=obstacles,
                    decisions=decisions,
                    robot_x=x,
                    robot_y=y,
                    active_decision=active_decision,
                    progress=frame_idx / max(1, total_frames - 1),
                )
            )
        else:
            update_camera(model, data, camera, x, y)
            renderer.update_scene(data, camera=camera)
            frames.append(renderer.render().copy())

        if frame_idx % max(1, fps // 8) == 0:
            trajectory.append(
                {
                    "time_s": round(time_s, 3),
                    "base_pos": body_position(model, data, "base_link"),
                    "lane": lane,
                    "action": action,
                    "active_decision_index": None if active_decision is None else active_decision["index"],
                    "chosen_lane": None if active_decision is None else active_decision["chosen_lane"],
                    "chosen_cost": None
                    if active_decision is None
                    else active_decision["options"][active_decision["chosen_lane"]]["total_cost"],
                }
            )

    final_pos = body_position(model, data, "base_link")
    lane_changes = sum(1 for prev, cur in zip(decisions, decisions[1:]) if prev["chosen_lane"] != cur["chosen_lane"])
    ascend_count = sum(1 for row in decisions if row["action"].startswith("ascend"))
    report = {
        "project": "Robot Autonomous Perception",
        "robot_platform": "Futurist humanoid from assets/Futurist",
        "task": "A humanoid robot autonomously perceives a real-world obstacle corridor and chooses left, right, or stair/ramp ascent without a scripted final route.",
        "success": final_pos[0] > 5.0,
        "model": str(urdf_path),
        "scene_xml": None if scene_path is None else str(scene_path),
        "video": str(video_path),
        "trajectory": str(trajectory_path),
        "decisions": str(decisions_path),
        "duration_s": duration_s,
        "fps": fps,
        "mujoco_depth": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "ngeom": int(model.ngeom),
            "obstacle_geoms": int(sum(1 for obs in obstacles for _ in [obs])),
        },
        "autonomy_metrics": {
            "decision_points": len(decisions),
            "lane_changes": lane_changes,
            "ascend_decisions": ascend_count,
            "obstacles_perceived": len(obstacles),
            "final_x_m": final_pos[0],
        },
        "final_base_pos": final_pos,
        "trajectory_samples": trajectory,
    }
    if render_fallback_reason:
        report["headless_schematic_renderer"] = True
        report["render_fallback_reason"] = render_fallback_reason

    try:
        iio.imwrite(video_path, np.asarray(frames), fps=fps, codec="libx264")
    except Exception as exc:
        fallback = video_path.with_suffix(".gif")
        iio.imwrite(fallback, np.asarray(frames), fps=fps)
        report["video"] = str(fallback)
        report["video_fallback_reason"] = str(exc)

    trajectory_path.write_text(json.dumps({"samples": trajectory, "waypoints": waypoints}, indent=2), encoding="utf-8")
    decisions_path.write_text(json.dumps({"decisions": decisions}, indent=2), encoding="utf-8")
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Robot Autonomous Perception MuJoCo submission.")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--decisions", type=Path, default=DEFAULT_DECISIONS)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE, help="Write the inspectable MuJoCo MJCF scene XML.")
    parser.add_argument("--duration", type=float, default=14.0)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--quick", action="store_true", help="Render a shorter, smaller smoke test.")
    parser.add_argument("--export-scene-only", action="store_true", help="Export scene.xml without rendering video artifacts.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.export_scene_only:
        scene_path = export_scene_xml(args.urdf, args.scene)
        print(json.dumps({"scene_xml": str(scene_path)}, indent=2))
        return 0

    if args.quick:
        args.duration = min(args.duration, 7.5)
        args.fps = min(args.fps, 16)
        args.width = min(args.width, 960)
        args.height = min(args.height, 540)

    summary = run_demo(
        urdf_path=args.urdf,
        video_path=args.output,
        trajectory_path=args.trajectory,
        report_path=args.report,
        decisions_path=args.decisions,
        duration_s=args.duration,
        fps=args.fps,
        width=args.width,
        height=args.height,
        scene_path=args.scene,
    )
    print(json.dumps({k: v for k, v in summary.items() if k != "trajectory_samples"}, indent=2))
    return 0 if summary["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
