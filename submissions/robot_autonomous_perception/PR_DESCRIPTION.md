# Robot Autonomous Perception

**Registration UUID:** `fd793b35-a006-4607-81de-339ec1bf4757`

## Project Name

**Robot Autonomous Perception** — Explainable Autonomous Humanoid Navigation in MuJoCo

## Robot Platform

Existing **Futurist humanoid robot** from `assets/Futurist/futurist.urdf`, imported into MuJoCo with a free base, procedural walking gait, lane surfaces, assembled 3D obstacles, route markers, lights, and a camera.

## Task Goal

Autonomously cross a realistic obstacle corridor without a hand-authored final route. At each checkpoint, the humanoid perceives nearby obstacles, scores left / center / right lane options, and chooses whether to walk clear, move left, move right, avoid blocked terrain, or ascend a passable stair/ramp.

## Core Features

- **Existing Futurist humanoid platform** loaded from the repository asset library.
- **Checked-in MuJoCo `scene.xml`** for direct MJCF inspection of the static robot/course world.
- **11 real-world obstacle types**: construction cones, split stairs, delivery crates, curbs, broken paver gap, traffic barrier, fallen sidewalk sign, service ramp, pipe cluster, and final stairs.
- **3-lane local perception cost map** for left / center / right route choices at each checkpoint.
- **Autonomous action selection**: walk clear, lane-change left, lane-change right, avoid gap/blockage, and ascend stairs/ramp.
- **11 decision points** with full option costs and chosen actions exported to JSON.
- **3 lane changes** and **2 ascent decisions** in the generated run.
- **14.0 second demo video at 24 FPS** with robot motion, obstacle field, colored scan choices, and active decision beacon.
- **MuJoCo scene depth**: 49 `nq`, 46 `nv`, 128 geoms, and 11 obstacle geoms.
- **Headless rendering support** through EGL, with a deterministic schematic fallback only if MuJoCo rendering is unavailable.
- **Validation package** with runner, requirements, registration metadata, report, trajectory, decision trace, judge brief, and rubric scorecard.

## One-Sentence Pitch

**Robot Autonomous Perception is an explainable MuJoCo humanoid navigation system that imports the provided Futurist robot, builds a realistic obstacle corridor, and autonomously chooses lane changes or stair/ramp ascent from local perception costs, completing an 11-checkpoint course with 3 lane changes, 2 ascent decisions, and a successful final position at 5.35004 m.**

---

# Why Robot Autonomous Perception Matters

Robotic autonomy is most convincing when the robot is not simply replaying a fixed path. A scripted route can look clean, but it does not demonstrate that the system understands why one lane is safer than another.

Robot Autonomous Perception explores a more transparent autonomy loop:

Instead of giving the humanoid a final route, the system gives it a world with recognizable obstacles and asks it to decide locally. At each checkpoint, the robot evaluates nearby geometry, assigns traversal costs, records every option, and selects the best lane/action pair.

The result is a reproducible MuJoCo submission where judges can inspect not only the video, but also the perception decisions that produced the route.

---

# Core Innovations

## 1. Explainable Perception-to-Action Navigation

Most simple obstacle demos hide the planner inside a preselected path:

> Place obstacles -> draw route -> animate robot along route.

Robot Autonomous Perception replaces that with an explicit decision loop.

### Local Perception Policy

The runner loads structured obstacle metadata and evaluates all three lanes at each checkpoint:

* Left lane
* Center lane
* Right lane

Each lane option records:

* Total cost
* Terrain cost
* Lateral movement cost
* Perceived obstacles
* Maximum observed height
* Maximum observed slope

### Autonomous Action Selector

The planner chooses the lowest-cost feasible action:

* Walk clear
* Move left
* Move right
* Avoid gap
* Avoid blocked obstacle
* Ascend stairs
* Traverse ramp

This turns the demo from path playback into an auditable perception-and-control pipeline.

---

## 2. Real-World Obstacle Semantics

The course is built from recognizable real-world hazards rather than anonymous boxes.

### Obstacle Field

* Construction cones / slalom posts
* Split stairs at 12 degrees
* Delivery crates
* Curb and gutter
* Broken paver gap
* Split stairs at 22 degrees
* Traffic barrier
* Fallen sidewalk sign
* Service ramp at 8 degrees
* Pipe cluster
* Final split stairs at 16 degrees

The planner treats these obstacles differently. High solid blocks become blocked lanes, gaps are avoided, curbs and ramps receive traversal penalties, and reasonable stairs can be climbed when lateral avoidance is not preferable.

---

## 3. Autonomous Lane and Terrain Decisions

The generated decision trace demonstrates multiple behaviors in one run:

1. Stay centered when the clear lane is cheapest.
2. Ascend the 12 degree split stairs instead of dodging.
3. Move left to avoid the broken paver gap.
4. Move right when left and center lanes are blocked.
5. Return center to avoid pipe clutter.
6. Ascend the final 16 degree stair set.

### Verified Outcome

* 11 decision points evaluated.
* 3 lane changes selected.
* 2 ascent actions selected.
* 11 obstacles perceived.
* Final base position reaches `x = 5.35004 m`.
* Run marked successful in the exported report.

---

## 4. Reproducible MuJoCo Evidence Package

The submission exports both visual and machine-readable evidence.

Artifacts include:

* Demo video
* Route trajectory
* Full decision trace
* Aggregate report
* Rubric scorecard
* Judge brief

This makes the autonomy inspectable from several angles: rendered behavior, numerical metrics, and per-checkpoint planner state.

---

# System Architecture

```text
Obstacle Metadata
       |
       v
 MuJoCo Course Builder
       |
       v
 Futurist Humanoid Scene
       |
       v
 Perception Checkpoints
       |
       v
 3-Lane Cost Map
       |
       v
 Autonomous Action Selector
       |
       v
 Route Waypoints + Gait Animation
       |
       v
 Video + Trajectory + Decision Artifacts
```

The planner closes the loop between perceived obstacle semantics and the humanoid route selected for the rendered run.

---

# Evaluation Results

## Generated Course Run

The exported report records a successful autonomous crossing.

| Metric | Result |
| ------ | -----: |
| Success | `true` |
| Duration | 14.0 s |
| FPS | 24 |
| Decision points | 11 |
| Lane changes | 3 |
| Ascent decisions | 2 |
| Obstacles perceived | 11 |
| Final X position | 5.35004 m |
| MuJoCo `nq` | 49 |
| MuJoCo `nv` | 46 |
| Total geoms | 128 |
| Obstacle geoms | 11 |

## Decision Behavior

| Course Segment | Perceived Challenge | Selected Behavior |
| -------------- | ------------------- | ----------------- |
| Construction cones | Left lane blocked by posts | Stay center |
| Split stairs, 12 degrees | Center lane traversable stairs | Ascend stairs |
| Broken paver gap | Center lane unsafe gap | Move left |
| Fallen sign + traffic barrier | Left and center blocked | Move right |
| Pipe cluster | Right lane cluttered | Return center |
| Final stairs, 16 degrees | Center lane traversable stairs | Ascend stairs |

The decision trace proves the robot is not following a fixed visual path. It records every lane option and the chosen action at every checkpoint.

---

# Verified Technical Achievements

| Capability | Verification |
| ---------- | ------------ |
| Existing robot asset usage | Loads `assets/Futurist/futurist.urdf` |
| MuJoCo scene construction | Provides checked-in `scene.xml` with free base, obstacle geoms, lane surfaces, lights, camera, and markers |
| Autonomous perception | Evaluates left / center / right lane costs at each checkpoint |
| Runtime decision making | Chooses walk, lane-change, or ascent actions from perceived terrain |
| Real-world task design | Uses cones, crates, curbs, gaps, barriers, signs, ramps, pipes, and stairs |
| Transparent artifacts | Exports video, trajectory, decisions, report, and rubric scorecard |
| Headless execution | Uses EGL automatically when no display is present |
| Reproducibility | Single Python runner, deterministic obstacle JSON, validator, and checked-in outputs |

---

# Validation Summary

The submission contains a local validator and all expected metadata/artifacts.

Important files:

```text
submissions/robot_autonomous_perception/
├── course_obstacles.json
├── scene.xml
├── run_robot_autonomous_perception.py
├── validate_submission.py
├── requirements.txt
├── registration.json
├── PR_DESCRIPTION.md
├── EVALUATION_GUIDE.md
├── rubric_scorecard.json
├── README.md
└── artifacts/
    ├── robot_autonomous_perception_demo.mp4
    ├── robot_autonomous_perception_trajectory.json
    ├── robot_autonomous_perception_decisions.json
    └── robot_autonomous_perception_report.json
```

Run from the repository root:

```bash
.venv/bin/python -m pip install -r submissions/robot_autonomous_perception/requirements.txt
.venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py
.venv/bin/python submissions/robot_autonomous_perception/validate_submission.py
```

Short smoke run:

```bash
.venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py --quick
```
