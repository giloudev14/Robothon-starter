# FF Master Tower Painting Descent MuJoCo Demo

A MuJoCo robotics demo where the repository's FF Master humanoid stands on a small exterior vehicle. The vehicle starts near the top of a building/tower and descends to the bottom landing while an autonomous contact-aware planner keeps a roller on the wall and paints a vertical strip.

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
- A receding-horizon mission planner that reads vehicle height, roller contact force, and current coverage, then decides whether to descend, hold for contact, or revisit weak swaths.
- Vehicle, floating-base robot, posture, and roller controllers that apply generalized forces through MuJoCo and step the physics state with `mj_step`.
- Roller-to-wall contact feedback read with `mj_contactForce`.
- Force-gated swept coverage tracking: paint swaths appear only when the roller has adequate measured normal force against the tower.
- A lightweight paint material approximation: visible paint swaths plus force-spawned droplets that drip and fade on the wall surface.
- A deterministic top-to-bottom painting descent, configurable with `--trips`, that prints a contact-aware coverage report.

The demo reuses the robot assets already present at the repository root, which keeps this submission small enough for AI review instead of vendoring a full external robot model tree.

Expected fast smoke-test evidence:

```text
contact-aware paint report: 13/14 swaths covered, ... contact steps, ... N average roller normal force
autonomy decisions: descend: coverage and contact acceptable; ...
```

The demo is intentionally compact and deterministic for judging. It is not a trained whole-body policy and does not claim full fluid simulation: the humanoid is still stabilized by simplified feedback/safety-harness controllers, and paint is a particle/drip material approximation rather than Navier-Stokes fluid. The scoring-critical behavior is nevertheless no longer just a visual script: descent decisions, roller force, and paint coverage are closed around MuJoCo state and contact feedback.
