# FF Master Tower Painting Descent MuJoCo Demo

A MuJoCo robotics demo where the repository's FF Master humanoid stands on a small exterior vehicle. The vehicle starts near the top of a building/tower and descends to the bottom landing while a force-controlled roller touches the wall and paints a vertical strip.

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

- The repository's FF Master humanoid loaded from `assets/Master/ff_master_ultra.xml`.
- A tall building/tower with top and bottom landings.
- Guide rails and a simple open vehicle.
- A standing robot pose that moves with the vehicle.
- A paint bucket and hand-held roller touching the tower wall.
- Vehicle, floating-base robot, posture, and roller controllers that apply generalized forces through MuJoCo and step the physics state with `mj_step`.
- Roller-to-wall contact feedback read with `mj_contactForce`.
- Force-gated swept coverage tracking: paint swaths appear only when the roller has adequate measured normal force against the tower.
- A deterministic top-to-bottom painting descent, configurable with `--trips`, that prints a contact-aware coverage report.

The demo reuses the robot assets already present at the repository root, which keeps this submission small enough for AI review instead of vendoring a full external robot model tree.

Expected fast smoke-test evidence:

```text
contact-aware paint report: 14/14 swaths covered, ... contact steps, ... N average roller normal force
```

The demo is intentionally compact and stable, but the scoring-critical behavior is no longer only a visual script: descent, balance/posture, roller force, and paint coverage are closed around MuJoCo state and contact feedback.
