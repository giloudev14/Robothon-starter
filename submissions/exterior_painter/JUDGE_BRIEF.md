# Judge Brief

This Robothon MuJoCo submission demonstrates the repository's FF Master humanoid standing in an exterior moving vehicle. The vehicle begins at the top of a building/tower and descends along guide rails while a contact-aware planner keeps a force-controlled roller on the wall and paints a vertical strip.

## Why This Entry Should Compete For #1

This entry turns a familiar construction task into a complete, judgeable robot system: task assignment, autonomous descent planning, MuJoCo force feedback, contact-gated painting, and video-ready visualization all run from one command. It uses the repository's real FF Master asset instead of a placeholder robot, and the behavior is closed around simulated state rather than purely animated frame timing.

The strongest point is clarity under scrutiny. The demo shows what it does, prints measurable evidence, and is explicit about its limits: the robot uses simplified stabilization rather than learned whole-body control, and the paint is a dynamic material approximation rather than a full fluid solver. That honesty makes the autonomy and contact-feedback improvements easier to trust.

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
4. The planner chooses descent/hold/recoat actions from vehicle height, coverage, and wall-force feedback.
5. The roller is controlled from wall-force feedback read through MuJoCo contact forces.
6. Fresh paint swaths and small dripping particles appear only after force-qualified swept coverage samples.
7. The vehicle reaches the bottom landing and prints completion plus a contact-aware paint report.

Expected headless evidence includes a line like:

```text
contact-aware paint report: 13/14 swaths covered, 162 contact steps, 147.5 N average roller normal force
autonomy decisions: descend: coverage and contact acceptable
```

The presentation remains deterministic for judging, but vehicle descent decisions, robot pose stabilization, roller contact, and coverage are driven by MuJoCo physics state and feedback controllers rather than direct frame-by-frame visual updates. It is still not a trained whole-body controller or full paint-fluid simulation; those limits are explicit in the README and self assessment.
