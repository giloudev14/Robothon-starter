from __future__ import annotations

import math
import tempfile
import textwrap
import time
from dataclasses import dataclass, field
from pathlib import Path

from .video import VideoRecorder


PROJECT_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = Path(__file__).resolve().parents[4]
FF_MASTER_XML = REPO_ROOT / "assets" / "Master" / "ff_master_ultra.xml"
DEFAULT_DESCENT_STEPS = 1
TOWER_X = 0.95
VEHICLE_X = 0.35
TOP_Z = 3.05
BOTTOM_Z = 0.55
TOWER_TOP_Z = 3.42
PAINT_SWATH_COUNT = 14
PAINT_SWATH_TOP_LOCAL_Z = 1.12
PAINT_SWATH_SPACING = 0.18
WALL_FACE_X = TOWER_X - 0.172
ROLLER_RELATIVE_X = WALL_FACE_X - VEHICLE_X - 0.012
ROLLER_RELATIVE_Z = -0.20
ROLLER_WORLD_X = WALL_FACE_X - 0.012
ROLLER_TARGET_FORCE_N = 8.0
ROLLER_MIN_PAINT_FORCE_N = 0.5
PAINT_COVERED_ALPHA = 0.80
DESCENT_RATE_M_PER_STEP = 0.0022
RECOAT_RATE_M_PER_STEP = 0.0024
PAINT_PARTICLE_COUNT = 56
_ACTIVE_RECORDER: VideoRecorder | None = None


@dataclass(frozen=True)
class DescentCommand:
    mode: str
    target_z: float
    reason: str


@dataclass(frozen=True)
class DescentSensors:
    vehicle_z: float
    roller_z: float
    normal_force: float
    coverage_ratio: float
    weak_swaths: tuple[int, ...]


@dataclass
class AutonomousPaintPlanner:
    direction: int = -1
    hold_steps: int = 0
    recoat_swath: int | None = None
    decisions: list[str] = field(default_factory=list)

    def command(self, sensors: DescentSensors) -> DescentCommand:
        if sensors.vehicle_z <= BOTTOM_Z + 0.035 and sensors.coverage_ratio >= 0.98:
            return DescentCommand("complete", BOTTOM_Z, "bottom reached with target coverage")

        if sensors.normal_force < ROLLER_MIN_PAINT_FORCE_N and sensors.coverage_ratio == 0.0 and self.hold_steps < 18:
            self.hold_steps += 1
            return self._remember("hold", sensors.vehicle_z, "waiting for roller contact")
        self.hold_steps = 0

        if sensors.vehicle_z <= BOTTOM_Z + 0.08 and sensors.weak_swaths:
            if self.recoat_swath is None:
                self.recoat_swath = sensors.weak_swaths[0]
            target_z = _vehicle_z_for_swath(self.recoat_swath)
            if abs(sensors.vehicle_z - target_z) < 0.06:
                self.recoat_swath = None
            step = RECOAT_RATE_M_PER_STEP if target_z > sensors.vehicle_z else -RECOAT_RATE_M_PER_STEP
            return self._remember(
                "recoat",
                _clamp(sensors.vehicle_z + step, BOTTOM_Z, TOP_Z),
                f"revisiting swath {self.recoat_swath if self.recoat_swath is not None else sensors.weak_swaths[0]:02d}",
            )

        target_z = _clamp(sensors.vehicle_z - DESCENT_RATE_M_PER_STEP, BOTTOM_Z, TOP_Z)
        return self._remember("descend", target_z, "coverage and contact acceptable")

    def _remember(self, mode: str, target_z: float, reason: str) -> DescentCommand:
        message = f"{mode}: {reason}"
        if not self.decisions or self.decisions[-1] != message:
            self.decisions.append(message)
        return DescentCommand(mode, target_z, reason)


@dataclass
class PaintParticle:
    z: float = 0.0
    y: float = 0.0
    alpha: float = 0.0
    drip_rate: float = 0.0


@dataclass
class PaintingState:
    swath_alpha: list[float] = field(default_factory=lambda: [0.0] * PAINT_SWATH_COUNT)
    painted_steps: int = 0
    contact_steps: int = 0
    force_samples: list[float] = field(default_factory=list)
    last_contact_z: float | None = None
    particles: list[PaintParticle] = field(default_factory=lambda: [PaintParticle() for _ in range(PAINT_PARTICLE_COUNT)])
    next_particle: int = 0
    planner_decisions: list[str] = field(default_factory=list)


def run_descent_demo(
    render: bool = False,
    speed: float = 1.0,
    video: str | None = None,
    trip_count: int = DEFAULT_DESCENT_STEPS,
) -> None:
    global _ACTIVE_RECORDER

    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError(
            "MuJoCo is not installed. Install it with: pip install -e '.[sim]'"
        ) from exc

    if not FF_MASTER_XML.exists():
        raise RuntimeError(
            "FF Master MuJoCo model was not found. Expected it at "
            f"{FF_MASTER_XML}. This repository should include assets/Master."
        )
    if trip_count < 1:
        raise ValueError("trip_count must be at least 1")

    scene_file = tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=FF_MASTER_XML.parent,
        prefix="tower_descent_",
        suffix=".xml",
        delete=False,
    )
    try:
        scene_path = Path(scene_file.name)
        scene_file.write(_scene_xml())
        scene_file.close()
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        _hide_paint_swaths(model)
        _set_standing_pose(model, data)
        _set_descent_height(model, data, TOP_Z)
        painting = PaintingState()

        viewer = None
        if render:
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(model, data)
        recorder = VideoRecorder(model, video, camera="descent_overview") if video else None
        previous_recorder = _ACTIVE_RECORDER
        _ACTIVE_RECORDER = recorder

        try:
            _sync(model, data, viewer)
            for trip in range(1, trip_count + 1):
                _descend_vehicle(model, data, speed=speed, viewer=viewer, painting=painting)
                print(f"painting descent complete {trip}/{trip_count}: vehicle reached the bottom landing")
                if trip < trip_count:
                    _return_to_top(model, data, speed=speed, viewer=viewer, painting=painting)
            coverage = sum(1 for alpha in painting.swath_alpha if alpha >= PAINT_COVERED_ALPHA)
            average_force = (
                sum(painting.force_samples) / len(painting.force_samples)
                if painting.force_samples
                else 0.0
            )
            print(
                "contact-aware paint report: "
                f"{coverage}/{PAINT_SWATH_COUNT} swaths covered, "
                f"{painting.contact_steps} contact steps, "
                f"{average_force:.1f} N average roller normal force"
            )
            if painting.planner_decisions:
                print("autonomy decisions: " + "; ".join(painting.planner_decisions[:6]))
            print("tower painting vehicle demo complete; close the MuJoCo viewer window after your screenshot")
            if recorder is not None:
                print(f"video saved: {recorder.path} ({recorder.frame_count} frames)")
            if viewer is not None:
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(model.opt.timestep)
        finally:
            _ACTIVE_RECORDER = previous_recorder
            if recorder is not None:
                recorder.close()
            if viewer is not None:
                viewer.close()
    finally:
        scene_file.close()
        Path(scene_file.name).unlink(missing_ok=True)


def _scene_xml() -> str:
    paint_swaths = "\n".join(
        f"""
              <geom name="paint_swath_{index:02d}" type="box"
                    pos="-0.173 0 {PAINT_SWATH_TOP_LOCAL_Z - index * PAINT_SWATH_SPACING:.3f}"
                    size="0.009 0.145 0.085"
                    rgba="0.92 0.94 0.78 1"
                    contype="0" conaffinity="0"/>
        """
        for index in range(PAINT_SWATH_COUNT)
    )
    paint_particles = "\n".join(
        f"""
              <geom name="paint_particle_{index:02d}" type="sphere"
                    pos="-0.185 0 0"
                    size="0.015"
                    rgba="0.92 0.94 0.78 0"
                    contype="0" conaffinity="0"/>
        """
        for index in range(PAINT_PARTICLE_COUNT)
    )
    return textwrap.dedent(
        f"""
        <mujoco model="ff_master_tower_descent">
          <include file="ff_master_ultra.xml"/>
          <compiler angle="radian"/>
          <option timestep="0.004" solver="CG" iterations="80" integrator="Euler"/>
          <statistic center="0.48 0 1.75" extent="4.0"/>
          <visual>
            <global offwidth="1280" offheight="720"/>
          </visual>
          <asset>
            <texture name="sky" type="skybox" builtin="gradient" rgb1="0.56 0.72 0.88" rgb2="0.92 0.95 0.97" width="512" height="512"/>
            <material name="ground_mat" rgba="0.24 0.26 0.25 1"/>
            <material name="tower_mat" rgba="0.62 0.63 0.60 1"/>
            <material name="window_mat" rgba="0.20 0.34 0.46 0.75"/>
            <material name="rail_mat" rgba="0.08 0.10 0.11 1"/>
            <material name="vehicle_mat" rgba="0.92 0.72 0.24 1"/>
            <material name="paint_bucket_mat" rgba="0.95 0.96 0.90 1"/>
            <material name="paint_mat" rgba="0.92 0.94 0.78 1"/>
            <material name="roller_mat" rgba="0.08 0.18 0.22 1"/>
            <material name="nap_mat" rgba="0.92 0.94 0.78 1"/>
          </asset>
          <contact>
            <pair geom1="paint_roller" geom2="tower_core" condim="3" friction="0.8 0.1 0.1" solref="0.01 1"/>
          </contact>
          <worldbody>
            <light name="sun" pos="-1.5 -2.0 5.0" dir="0.5 0.5 -1.0" directional="true" diffuse="0.9 0.9 0.85"/>
            <camera name="descent_overview" pos="-2.7 -4.4 2.65" xyaxes="0.86 -0.51 0 0.24 0.41 0.88" fovy="68"/>
            <geom name="ground" type="plane" size="4.0 4.0 0.02" material="ground_mat"/>

            <body name="tower" pos="{TOWER_X} 0 1.7">
              <geom name="tower_core" type="box" size="0.16 0.48 1.7" material="tower_mat"/>
              <geom name="window_01" type="box" pos="-0.165 -0.23 0.82" size="0.012 0.08 0.16" material="window_mat"/>
              <geom name="window_02" type="box" pos="-0.165 0.23 0.82" size="0.012 0.08 0.16" material="window_mat"/>
              <geom name="window_03" type="box" pos="-0.165 -0.23 0.26" size="0.012 0.08 0.16" material="window_mat"/>
              <geom name="window_04" type="box" pos="-0.165 0.23 0.26" size="0.012 0.08 0.16" material="window_mat"/>
              <geom name="window_05" type="box" pos="-0.165 -0.23 -0.30" size="0.012 0.08 0.16" material="window_mat"/>
              <geom name="window_06" type="box" pos="-0.165 0.23 -0.30" size="0.012 0.08 0.16" material="window_mat"/>
              {paint_swaths}
              {paint_particles}
            </body>

            <geom name="left_guide_rail" type="capsule" fromto="{VEHICLE_X} -0.36 {BOTTOM_Z} {VEHICLE_X} -0.36 {TOP_Z}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>
            <geom name="right_guide_rail" type="capsule" fromto="{VEHICLE_X} 0.36 {BOTTOM_Z} {VEHICLE_X} 0.36 {TOP_Z}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>
            <geom name="left_top_rail_curve" type="capsule" fromto="{VEHICLE_X:.3f} -0.36 {TOP_Z:.3f} {VEHICLE_X + 0.08:.3f} -0.36 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>
            <geom name="right_top_rail_curve" type="capsule" fromto="{VEHICLE_X:.3f} 0.36 {TOP_Z:.3f} {VEHICLE_X + 0.08:.3f} 0.36 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>
            <geom name="left_top_roof_anchor" type="capsule" fromto="{VEHICLE_X + 0.08:.3f} -0.36 {TOWER_TOP_Z:.3f} {TOWER_X:.3f} -0.31 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>
            <geom name="right_top_roof_anchor" type="capsule" fromto="{VEHICLE_X + 0.08:.3f} 0.36 {TOWER_TOP_Z:.3f} {TOWER_X:.3f} 0.31 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat" contype="0" conaffinity="0"/>

            <body name="descent_vehicle" pos="{VEHICLE_X} 0 {TOP_Z}">
              <freejoint name="descent_vehicle_free"/>
              <geom name="vehicle_floor" type="box" pos="0 0 0" size="0.34 0.34 0.035" material="vehicle_mat" mass="8"/>
              <geom name="vehicle_back" type="box" pos="0.12 0 0.24" size="0.035 0.32 0.24" material="vehicle_mat" mass="3"/>
              <geom name="vehicle_left_guard" type="box" pos="-0.02 -0.34 0.18" size="0.30 0.025 0.18" material="vehicle_mat" mass="2"/>
              <geom name="vehicle_right_guard" type="box" pos="-0.02 0.34 0.18" size="0.30 0.025 0.18" material="vehicle_mat" mass="2"/>
              <geom name="paint_bucket" type="cylinder" pos="-0.12 -0.18 0.14" size="0.07 0.11" material="paint_bucket_mat" mass="1"/>
              <geom name="paint_surface" type="cylinder" pos="-0.12 -0.18 0.23" size="0.058 0.01" material="paint_mat" mass="0.1"/>
            </body>

            <body name="paint_roller_head" pos="{ROLLER_WORLD_X:.3f} 0 {TOP_Z + ROLLER_RELATIVE_Z:.3f}">
              <freejoint name="paint_roller_head_free"/>
              <geom name="paint_roller" type="cylinder" fromto="0 -0.16 0 0 0.16 0" size="0.035" material="nap_mat" mass="0.4"/>
            </body>

            <body name="roller_handle_body" pos="0 0 1">
              <freejoint name="roller_handle_free"/>
              <geom name="roller_handle" type="capsule" size="0.014 0.5" material="roller_mat" mass="0.2" contype="0" conaffinity="0"/>
            </body>

          </worldbody>
        </mujoco>
        """
    )


def _descend_vehicle(
    model: object,
    data: object,
    speed: float,
    viewer: object | None,
    painting: PaintingState,
) -> None:
    _move_vehicle(model, data, TOP_Z, BOTTOM_Z, frames=_frames(520, speed), viewer=viewer, painting=painting)


def _return_to_top(
    model: object,
    data: object,
    speed: float,
    viewer: object | None,
    painting: PaintingState,
) -> None:
    _move_vehicle(model, data, BOTTOM_Z, TOP_Z, frames=_frames(160, speed), viewer=viewer, painting=painting)


def _move_vehicle(
    model: object,
    data: object,
    start_z: float,
    end_z: float,
    frames: int,
    viewer: object | None,
    painting: PaintingState,
) -> None:
    if end_z < start_z:
        planner = AutonomousPaintPlanner()
        for _ in range(max(1, frames * 4)):
            sensors = _sense_descent_state(model, data, painting)
            command = planner.command(sensors)
            _apply_descent_controllers(model, data, command.target_z, True, painting)
            _sync(model, data, viewer)
            if command.mode == "complete":
                break
        painting.planner_decisions = planner.decisions
        return

    for _ in range(max(1, frames)):
        current_z = _freejoint_position(model, data, "descent_vehicle_free")[2]
        target_z = _clamp(current_z + RECOAT_RATE_M_PER_STEP * 2.5, BOTTOM_Z, TOP_Z)
        _apply_descent_controllers(model, data, target_z, False, painting)
        _sync(model, data, viewer)


def _apply_descent_controllers(
    model: object,
    data: object,
    vehicle_z: float,
    painting_enabled: bool,
    painting: PaintingState,
) -> None:
    data.qfrc_applied[:] = 0.0
    vehicle_pos = _freejoint_position(model, data, "descent_vehicle_free")
    _apply_freejoint_position_control(
        model,
        data,
        "descent_vehicle_free",
        (VEHICLE_X, 0.0, vehicle_z),
        kp=2200.0,
        kd=260.0,
        max_force=5200.0,
        gravity_body="descent_vehicle",
    )
    _apply_freejoint_position_control(
        model,
        data,
        "floating_base_joint",
        (VEHICLE_X - 0.10, 0.0, vehicle_pos[2] + 0.84),
        kp=900.0,
        kd=110.0,
        max_force=1200.0,
        gravity_body="pelvis",
    )
    _apply_standing_posture_control(model, data)

    normal_force = _roller_wall_force(model, data)
    roller_x = WALL_FACE_X - 0.034 + 0.000005 * (ROLLER_TARGET_FORCE_N - normal_force)
    roller_x = max(WALL_FACE_X - 0.038, min(WALL_FACE_X - 0.031, roller_x))
    _apply_freejoint_position_control(
        model,
        data,
        "paint_roller_head_free",
        (roller_x, 0.0, vehicle_pos[2] + ROLLER_RELATIVE_Z),
        kp=160.0,
        kd=40.0,
        max_force=80.0,
        gravity_body="paint_roller_head",
    )

    if normal_force > 0.05:
        painting.contact_steps += 1
        painting.force_samples.append(normal_force)
    if painting_enabled:
        _update_paint_swaths(model, data, painting, normal_force)
    _update_paint_particles(model, data, painting, normal_force, painting_enabled)

    _align_paint_tool_to_hand(model, data, vehicle_pos[2])


def _set_descent_height(model: object, data: object, vehicle_z: float) -> None:
    _set_freejoint_pose(model, data, "descent_vehicle_free", (VEHICLE_X, 0.0, vehicle_z), yaw=0.0)
    _set_freejoint_pose(model, data, "floating_base_joint", (VEHICLE_X - 0.10, 0.0, vehicle_z + 0.84), yaw=0.0)
    _set_standing_pose(model, data)
    _align_paint_tool_to_hand(model, data, vehicle_z)


def _set_standing_pose(model: object, data: object) -> None:
    for joint_name, value in _standing_pose_targets().items():
        _set_joint_qpos(model, data, joint_name, value)


def _standing_pose_targets() -> dict[str, float]:
    return {
        "left_hip_pitch_joint": -0.08,
        "right_hip_pitch_joint": -0.08,
        "left_knee_joint": 0.18,
        "right_knee_joint": 0.18,
        "left_ankle_pitch_joint": -0.08,
        "right_ankle_pitch_joint": -0.08,
        "waist_pitch_joint": 0.08,
        "waist_yaw_joint": 0.18,
        "left_shoulder_pitch_joint": 0.20,
        "right_shoulder_pitch_joint": -0.18,
        "left_shoulder_roll_joint": 0.24,
        "right_shoulder_roll_joint": -1.34,
        "left_elbow_joint": 0.45,
        "right_elbow_joint": 0.08,
        "right_wrist_roll_joint": 0.08,
        "right_wrist_pitch_joint": -0.08,
        "right_wrist_yaw_joint": 0.08,
    }


def _apply_standing_posture_control(model: object, data: object) -> None:
    for joint_name, target in _standing_pose_targets().items():
        joint_id = model.joint(joint_name).id
        qpos_address = model.jnt_qposadr[joint_id]
        dof_address = model.jnt_dofadr[joint_id]
        error = target - float(data.qpos[qpos_address])
        velocity = float(data.qvel[dof_address])
        data.qfrc_applied[dof_address] += max(-35.0, min(35.0, 95.0 * error - 8.0 * velocity))


def _align_paint_tool_to_hand(model: object, data: object, vehicle_z: float) -> None:
    import mujoco

    roller_center = _freejoint_position(model, data, "paint_roller_head_free")
    mujoco.mj_forward(model, data)

    hand_grip = _right_hand_grip_position(model, data)
    _set_capsule_between(model, data, "roller_handle_free", "roller_handle", hand_grip, roller_center)


def _right_hand_grip_position(model: object, data: object) -> tuple[float, float, float]:
    body_id = model.body("right_wrist_roll_link").id
    wrist = tuple(float(value) for value in data.xpos[body_id])
    return (wrist[0] - 0.05, wrist[1], wrist[2] - 0.12)


def _hide_paint_swaths(model: object) -> None:
    for index in range(PAINT_SWATH_COUNT):
        geom_id = model.geom(f"paint_swath_{index:02d}").id
        model.geom_rgba[geom_id, 3] = 0.0
    for index in range(PAINT_PARTICLE_COUNT):
        geom_id = model.geom(f"paint_particle_{index:02d}").id
        model.geom_rgba[geom_id, 3] = 0.0


def _sense_descent_state(model: object, data: object, painting: PaintingState) -> DescentSensors:
    vehicle_z = _freejoint_position(model, data, "descent_vehicle_free")[2]
    roller_z = _freejoint_position(model, data, "paint_roller_head_free")[2]
    covered = sum(1 for alpha in painting.swath_alpha if alpha >= PAINT_COVERED_ALPHA)
    coverage_ratio = covered / PAINT_SWATH_COUNT
    weak_swaths = tuple(index for index, alpha in enumerate(painting.swath_alpha) if alpha < PAINT_COVERED_ALPHA)
    return DescentSensors(
        vehicle_z=vehicle_z,
        roller_z=roller_z,
        normal_force=_roller_wall_force(model, data),
        coverage_ratio=coverage_ratio,
        weak_swaths=weak_swaths,
    )


def _update_paint_swaths(
    model: object,
    data: object,
    painting: PaintingState,
    normal_force: float,
) -> None:
    roller_world_z = _freejoint_position(model, data, "paint_roller_head_free")[2]
    force_quality = max(0.0, min(1.0, normal_force / ROLLER_TARGET_FORCE_N))
    if normal_force >= ROLLER_MIN_PAINT_FORCE_N and painting.last_contact_z is not None:
        swept_min = min(roller_world_z, painting.last_contact_z) - 0.30
        swept_max = max(roller_world_z, painting.last_contact_z) + 0.30
    else:
        swept_min = roller_world_z - 0.30
        swept_max = roller_world_z + 0.30
    for index in range(PAINT_SWATH_COUNT):
        geom_id = model.geom(f"paint_swath_{index:02d}").id
        swath_world_z = 1.7 + PAINT_SWATH_TOP_LOCAL_Z - index * PAINT_SWATH_SPACING
        overlap = swept_min <= swath_world_z <= swept_max
        if overlap and normal_force >= ROLLER_MIN_PAINT_FORCE_N:
            painting.swath_alpha[index] = min(1.0, painting.swath_alpha[index] + force_quality)
            painting.painted_steps += 1
        model.geom_rgba[geom_id, 3] = painting.swath_alpha[index]
    if normal_force >= ROLLER_MIN_PAINT_FORCE_N:
        painting.last_contact_z = roller_world_z


def _update_paint_particles(
    model: object,
    data: object,
    painting: PaintingState,
    normal_force: float,
    painting_enabled: bool,
) -> None:
    roller_world_z = _freejoint_position(model, data, "paint_roller_head_free")[2]
    if painting_enabled and normal_force >= ROLLER_MIN_PAINT_FORCE_N:
        force_quality = max(0.0, min(1.0, normal_force / ROLLER_TARGET_FORCE_N))
        for offset in (-0.09, 0.0, 0.09):
            particle = painting.particles[painting.next_particle]
            particle.z = roller_world_z + 0.035 * math.sin(painting.painted_steps + offset * 20.0)
            particle.y = offset
            particle.alpha = min(1.0, 0.35 + 0.6 * force_quality)
            particle.drip_rate = 0.0004 + 0.0016 * force_quality
            painting.next_particle = (painting.next_particle + 1) % PAINT_PARTICLE_COUNT

    for index, particle in enumerate(painting.particles):
        geom_id = model.geom(f"paint_particle_{index:02d}").id
        if particle.alpha <= 0.01:
            model.geom_rgba[geom_id, 3] = 0.0
            continue
        particle.z = max(BOTTOM_Z - 0.08, particle.z - particle.drip_rate)
        particle.alpha *= 0.996
        model.geom_pos[geom_id] = (-0.185, particle.y, particle.z - 1.7)
        model.geom_rgba[geom_id, 3] = particle.alpha


def _vehicle_z_for_swath(index: int) -> float:
    swath_world_z = 1.7 + PAINT_SWATH_TOP_LOCAL_Z - index * PAINT_SWATH_SPACING
    return _clamp(swath_world_z - ROLLER_RELATIVE_Z, BOTTOM_Z, TOP_Z)


def _roller_wall_force(model: object, data: object) -> float:
    import mujoco
    import numpy as np

    roller_id = model.geom("paint_roller").id
    wall_ids = {model.geom("tower_core").id}
    force = np.zeros(6)
    total = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        if {contact.geom1, contact.geom2} == {roller_id, *wall_ids}:
            mujoco.mj_contactForce(model, data, index, force)
            total += abs(float(force[0]))
    return total


def _apply_freejoint_position_control(
    model: object,
    data: object,
    joint_name: str,
    target: tuple[float, float, float],
    kp: float,
    kd: float,
    max_force: float,
    gravity_body: str | None = None,
) -> None:
    qpos_address = _freejoint_qpos_address(model, joint_name)
    dof_address = _freejoint_dof_address(model, joint_name)
    for axis in range(3):
        error = target[axis] - float(data.qpos[qpos_address + axis])
        velocity = float(data.qvel[dof_address + axis])
        force = kp * error - kd * velocity
        data.qfrc_applied[dof_address + axis] += max(-max_force, min(max_force, force))
    if gravity_body is not None:
        body_id = model.body(gravity_body).id
        data.qfrc_applied[dof_address + 2] += float(model.body_subtreemass[body_id]) * abs(float(model.opt.gravity[2]))
    yaw = _freejoint_yaw(model, data, joint_name)
    yaw_rate = float(data.qvel[dof_address + 5])
    data.qfrc_applied[dof_address + 5] += max(-80.0, min(80.0, -120.0 * yaw - 12.0 * yaw_rate))


def _freejoint_position(model: object, data: object, joint_name: str) -> tuple[float, float, float]:
    qpos_address = _freejoint_qpos_address(model, joint_name)
    return tuple(float(value) for value in data.qpos[qpos_address : qpos_address + 3])


def _freejoint_yaw(model: object, data: object, joint_name: str) -> float:
    qpos_address = _freejoint_qpos_address(model, joint_name)
    w, x, y, z = (float(value) for value in data.qpos[qpos_address + 3 : qpos_address + 7])
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _freejoint_qpos_address(model: object, joint_name: str) -> int:
    joint_id = model.joint(joint_name).id
    return int(model.jnt_qposadr[joint_id])


def _freejoint_dof_address(model: object, joint_name: str) -> int:
    joint_id = model.joint(joint_name).id
    return int(model.jnt_dofadr[joint_id])


def _set_freejoint_pose(
    model: object,
    data: object,
    joint_name: str,
    position: tuple[float, float, float],
    yaw: float,
) -> None:
    joint_id = model.joint(joint_name).id
    qpos_address = model.jnt_qposadr[joint_id]
    data.qpos[qpos_address : qpos_address + 3] = position
    data.qpos[qpos_address + 3 : qpos_address + 7] = _yaw_quat(yaw)


def _set_capsule_between(
    model: object,
    data: object,
    joint_name: str,
    geom_name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> None:
    vector = tuple(end[index] - start[index] for index in range(3))
    length = math.sqrt(sum(component * component for component in vector))
    if length <= 1e-6:
        return
    midpoint = tuple((start[index] + end[index]) * 0.5 for index in range(3))
    joint_id = model.joint(joint_name).id
    qpos_address = model.jnt_qposadr[joint_id]
    data.qpos[qpos_address : qpos_address + 3] = midpoint
    data.qpos[qpos_address + 3 : qpos_address + 7] = _quat_from_z_axis(vector)
    geom_id = model.geom(geom_name).id
    model.geom_size[geom_id, 1] = length * 0.5


def _set_joint_qpos(model: object, data: object, joint_name: str, value: float) -> None:
    joint_id = model.joint(joint_name).id
    qpos_address = model.jnt_qposadr[joint_id]
    data.qpos[qpos_address] = value


def _sync(model: object, data: object, viewer: object | None) -> None:
    import mujoco

    mujoco.mj_step(model, data)
    if viewer is not None:
        viewer.sync()
        time.sleep(model.opt.timestep)
    if _ACTIVE_RECORDER is not None:
        _ACTIVE_RECORDER.record(data)


def _frames(base_frames: int, speed: float) -> int:
    return max(520, int(base_frames / max(speed, 0.05)))


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def _quat_from_z_axis(vector: tuple[float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    target = tuple(component / length for component in vector)
    source = (0.0, 0.0, 1.0)
    dot = max(-1.0, min(1.0, sum(source[index] * target[index] for index in range(3))))
    if dot > 0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    if dot < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)
    cross = (
        source[1] * target[2] - source[2] * target[1],
        source[2] * target[0] - source[0] * target[2],
        source[0] * target[1] - source[1] * target[0],
    )
    quat = (1.0 + dot, cross[0], cross[1], cross[2])
    norm = math.sqrt(sum(component * component for component in quat))
    return tuple(component / norm for component in quat)
