# Unitree G1 Tower Painting Descent MuJoCo Demo

A simple MuJoCo robotics demo where a Unitree G1 humanoid stands on a small moving exterior vehicle. The vehicle starts near the top of a building/tower and descends to the bottom landing while a roller touches the wall and paints a vertical strip.

## Quick Start

```bash
cd submissions/exterior_painter
python3 -m venv .venv
.venv/bin/pip install -e '.[sim]'
```

Run the painting descent demo with the MuJoCo viewer:

```bash
.venv/bin/python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --render
```

Run it headlessly:

```bash
.venv/bin/python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 8
```

Save a video:

```bash
MUJOCO_GL=egl .venv/bin/python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 8 --video tower_painting_descent.mp4
```

## Behavior

The generated scene contains:

- A Unitree G1 humanoid loaded from the official Menagerie `g1_with_hands.xml`.
- A tall building/tower with top and bottom landings.
- Guide rails and a simple open vehicle.
- A standing robot pose that moves with the vehicle.
- A paint bucket and hand-held roller touching the tower wall.
- Articulated right-hand fingers closed around the roller handle.
- Progressive fresh paint swaths that appear as the vehicle moves down.
- A deterministic top-to-bottom painting descent, configurable with `--trips`.

The demo is intentionally simple and stable: each frame directly updates the vehicle, standing robot pose, roller contact, and paint coverage, so the visual story is easy to inspect in the viewer or in a headless MP4.
