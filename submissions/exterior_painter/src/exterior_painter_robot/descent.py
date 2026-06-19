from __future__ import annotations

import math
import tempfile
import textwrap
import time
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
ROLLER_RELATIVE_Z = 0.78
ROLLER_WORLD_X = WALL_FACE_X - 0.012
_ACTIVE_RECORDER: VideoRecorder | None = None


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
                _descend_vehicle(model, data, speed=speed, viewer=viewer)
                print(f"painting descent complete {trip}/{trip_count}: vehicle reached the bottom landing")
                if trip < trip_count:
                    _return_to_top(model, data, speed=speed, viewer=viewer)
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
            </body>

            <geom name="left_guide_rail" type="capsule" fromto="{VEHICLE_X} -0.36 {BOTTOM_Z} {VEHICLE_X} -0.36 {TOP_Z}" size="0.018" material="rail_mat"/>
            <geom name="right_guide_rail" type="capsule" fromto="{VEHICLE_X} 0.36 {BOTTOM_Z} {VEHICLE_X} 0.36 {TOP_Z}" size="0.018" material="rail_mat"/>
            <geom name="left_top_rail_curve" type="capsule" fromto="{VEHICLE_X:.3f} -0.36 {TOP_Z:.3f} {VEHICLE_X + 0.08:.3f} -0.36 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat"/>
            <geom name="right_top_rail_curve" type="capsule" fromto="{VEHICLE_X:.3f} 0.36 {TOP_Z:.3f} {VEHICLE_X + 0.08:.3f} 0.36 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat"/>
            <geom name="left_top_roof_anchor" type="capsule" fromto="{VEHICLE_X + 0.08:.3f} -0.36 {TOWER_TOP_Z:.3f} {TOWER_X:.3f} -0.31 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat"/>
            <geom name="right_top_roof_anchor" type="capsule" fromto="{VEHICLE_X + 0.08:.3f} 0.36 {TOWER_TOP_Z:.3f} {TOWER_X:.3f} 0.31 {TOWER_TOP_Z:.3f}" size="0.018" material="rail_mat"/>

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
              <geom name="roller_handle" type="capsule" size="0.014 0.5" material="roller_mat" mass="0.2"/>
            </body>

          </worldbody>
        </mujoco>
        """
    )


def _descend_vehicle(model: object, data: object, speed: float, viewer: object | None) -> None:
    _move_vehicle(model, data, TOP_Z, BOTTOM_Z, frames=_frames(520, speed), viewer=viewer)


def _return_to_top(model: object, data: object, speed: float, viewer: object | None) -> None:
    _move_vehicle(model, data, BOTTOM_Z, TOP_Z, frames=_frames(160, speed), viewer=viewer)


def _move_vehicle(
    model: object,
    data: object,
    start_z: float,
    end_z: float,
    frames: int,
    viewer: object | None,
) -> None:
    for frame in range(max(1, frames)):
        alpha = (frame + 1) / max(1, frames)
        eased = 0.5 - 0.5 * math.cos(alpha * math.pi)
        z = start_z + (end_z - start_z) * eased
        _set_descent_height(model, data, z)
        _sync(model, data, viewer)


def _set_descent_height(model: object, data: object, vehicle_z: float) -> None:
    _set_freejoint_pose(model, data, "descent_vehicle_free", (VEHICLE_X, 0.0, vehicle_z), yaw=0.0)
    _set_freejoint_pose(model, data, "floating_base_joint", (VEHICLE_X - 0.10, 0.0, vehicle_z + 0.84), yaw=0.0)
    _set_standing_pose(model, data)
    _align_paint_tool_to_hand(model, data, vehicle_z)
    _update_paint_swaths(model, vehicle_z)


def _set_standing_pose(model: object, data: object) -> None:
    standing_pose = {
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
    for joint_name, value in standing_pose.items():
        _set_joint_qpos(model, data, joint_name, value)


def _align_paint_tool_to_hand(model: object, data: object, vehicle_z: float) -> None:
    import mujoco

    roller_center = (ROLLER_WORLD_X, 0.0, vehicle_z + ROLLER_RELATIVE_Z)
    _set_freejoint_pose(model, data, "paint_roller_head_free", roller_center, yaw=0.0)
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


def _update_paint_swaths(model: object, vehicle_z: float) -> None:
    roller_world_z = vehicle_z + ROLLER_RELATIVE_Z
    for index in range(PAINT_SWATH_COUNT):
        geom_id = model.geom(f"paint_swath_{index:02d}").id
        swath_world_z = 1.7 + PAINT_SWATH_TOP_LOCAL_Z - index * PAINT_SWATH_SPACING
        already_passed = roller_world_z < swath_world_z - 0.035
        model.geom_rgba[geom_id, 3] = 1.0 if already_passed else 0.0


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

    mujoco.mj_forward(model, data)
    if viewer is not None:
        viewer.sync()
    if _ACTIVE_RECORDER is not None:
        _ACTIVE_RECORDER.record(data)
    time.sleep(model.opt.timestep)


def _frames(base_frames: int, speed: float) -> int:
    return max(1, int(base_frames / max(speed, 0.05)))


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
