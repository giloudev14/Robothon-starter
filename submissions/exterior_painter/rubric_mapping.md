# Rubric Mapping

## Runnability

- Main command: `python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --render`.
- Headless command: `MUJOCO_GL=egl python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 8 --video tower_painting_descent.mp4`.
- Fast smoke test: `python -m exterior_painter_robot examples/tiny_house_plan.yaml --owner-goal "paint the tower while riding down" --descent --speed 50`.

## Task Design

- Clear task: a robot stands in a vehicle and paints the tower wall while descending from top to bottom.
- Clear success state: the vehicle reaches the bottom landing with visible paint coverage on the wall.

## Engineering

- Uses the official Menagerie `g1_with_hands.xml` model.
- Generates a task-specific MJCF scene at runtime.
- Cleans up temporary XML after execution.
- Supports viewer and video modes.
