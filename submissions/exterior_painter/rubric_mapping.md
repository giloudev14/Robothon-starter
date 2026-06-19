# Rubric Mapping

## Runnability

- Main command: `python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --render`.
- Headless command: `MUJOCO_GL=egl python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 8 --video tower_painting_descent.mp4`.
- Fast smoke test: `python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 50`.
- Expected smoke-test evidence: `contact-aware paint report: 14/14 swaths covered, ... contact steps, ... N average roller normal force`.

## Task Design

- Clear task: a robot stands in a vehicle and paints the tower wall while descending from top to bottom.
- Clear success state: the vehicle reaches the bottom landing with force-qualified visible paint coverage on the wall.

## Engineering

- Uses the repository's existing FF Master `assets/Master/ff_master_ultra.xml` model.
- Generates a task-specific MJCF scene at runtime.
- Drives vehicle descent, FF Master floating-base stabilization, joint posture, and roller placement with generalized-force feedback controllers.
- Reads MuJoCo roller/wall contact forces with `mj_contactForce`.
- Paint coverage is not time-triggered: it is updated by swept roller coverage only when measured normal force exceeds the painting threshold.
- Cleans up temporary XML after execution.
- Supports viewer and video modes.
