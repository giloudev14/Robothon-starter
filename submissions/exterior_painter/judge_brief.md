# Judge Brief

This Robothon MuJoCo submission demonstrates the repository's FF Master humanoid standing in an exterior moving vehicle. The vehicle begins at the top of a building/tower and descends along guide rails while a force-controlled roller touches the wall and paints a vertical strip.

## Run

```bash
cd submissions/exterior_painter
pip install -e '.[sim]'
python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --render
```

Headless video:

```bash
MUJOCO_GL=egl python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 8 --video tower_painting_descent.mp4
```

## What To Look For

1. The FF Master starts standing in the vehicle near the tower top.
2. The vehicle moves downward along the exterior rails.
3. A paint bucket is on the vehicle and the robot's right wrist stays aligned to the roller handle.
4. The roller is controlled from wall-force feedback read through MuJoCo contact forces.
5. Fresh paint swaths appear only after force-qualified swept coverage samples.
6. The vehicle reaches the bottom landing and prints completion plus a contact-aware paint report.

Expected headless evidence includes a line like:

```text
contact-aware paint report: 14/14 swaths covered, 145 contact steps, 100.5 N average roller normal force
```

The presentation remains deterministic for judging, but vehicle descent, robot pose stabilization, roller contact, and coverage are now driven by MuJoCo physics state and feedback controllers rather than direct frame-by-frame visual updates.
