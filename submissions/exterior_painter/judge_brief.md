# Judge Brief

This Robothon MuJoCo submission demonstrates a plain Unitree G1 humanoid standing in an exterior moving vehicle. The vehicle begins at the top of a building/tower and descends along guide rails while a roller touches the wall and paints a vertical strip.

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

1. The G1 starts standing in the vehicle near the tower top.
2. The vehicle moves downward along the exterior rails.
3. A paint bucket is on the vehicle and the robot's articulated right-hand fingers close around the roller handle.
4. The roller stays against the tower wall as the vehicle travels.
5. Fresh paint swaths appear progressively down the wall.
6. The vehicle reaches the bottom landing and prints completion.

The motion is scripted for stability and clarity.
