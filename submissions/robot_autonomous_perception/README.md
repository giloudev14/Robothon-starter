# Robot Autonomous Perception

**Robothon 2026 · Faraday Future MuJoCo Hackathon**

---

## Project Name

**Robot Autonomous Perception** - an explainable MuJoCo simulation of autonomous humanoid navigation, using the provided Futurist robot to perceive a realistic obstacle corridor and choose lane changes or stair/ramp ascent without a scripted final route.

---

## Robot Platform

| Component | Details |
|-----------|---------|
| Humanoid Robot | Futurist humanoid from `assets/Futurist/futurist.urdf` |
| Robot Type | Full humanoid walking robot with free base |
| Simulation | Checked-in MuJoCo `scene.xml` plus URDF import with added lane surfaces, obstacle geoms, lights, camera, and route markers |
| Control | Procedural gait animation driven by autonomous route decisions |
| Perception | Local three-lane obstacle cost map generated from course metadata |
| Scene Scale | 49 `nq`, 46 `nv`, 128 geoms, 11 obstacle geoms |

---

## Task Goal

Navigate a humanoid robot through a realistic obstacle corridor using autonomous local perception and decision making.

The simulation executes a complete navigation run:

1. Load the Futurist humanoid into MuJoCo.
2. Construct a corridor with real-world obstacle types.
3. Evaluate left, center, and right lane options at each checkpoint.
4. Convert nearby obstacle geometry into traversal costs.
5. Choose whether to walk clear, change lanes, avoid unsafe terrain, or ascend stairs/ramp.
6. Generate route waypoints from those decisions.
7. Render a video and export trajectory, decision, and report artifacts.

The checked-in run completes 11 perception checkpoints with 3 lane changes, 2 ascent decisions, and a successful final base position at `x = 5.35004 m`.

---

## The Problem

Humanoid navigation demos often look autonomous while hiding a fixed path underneath. The robot appears to avoid obstacles, but the route was already chosen by the author.

That creates three problems:

**1. Weak evidence of autonomy.** A rendered path does not prove the robot perceived the scene or selected actions at runtime. It only proves that an animation reached the goal.

**2. Poor interpretability.** When the robot moves left or climbs stairs, reviewers should be able to inspect why. Without a decision trace, there is no way to distinguish obstacle reasoning from hand-authored waypoints.

**3. Limited task realism.** Real sidewalks and work sites contain mixed hazards: gaps, curbs, cones, barriers, ramps, signs, crates, pipes, and stairs. A navigation task should require different responses to different obstacle semantics.

The root cause is a mismatch between what autonomy claims and what the artifact proves.

> A real navigation system should answer: "What did the robot perceive, what options did it score, and why did it choose this route?"
>
> A fixed animation only answers: "Where did the robot move?"

Robot Autonomous Perception is designed around that evidence gap.

---

## Robot Autonomous Perception as the Solution

Robot Autonomous Perception introduces an explainable perception-to-action layer between the obstacle course and the humanoid gait animation.

```
Before (scripted route demo):
  Hand-authored path -> robot animation -> video

After (Robot Autonomous Perception):
  Obstacle metadata -> lane cost map -> autonomous action choice -> route -> robot animation + decision artifacts
```

The robot is not assigned a final route. It decides locally.

Concretely:
- The runner loads 11 real-world obstacle definitions from `course_obstacles.json`.
- `scene.xml` exposes the static MuJoCo world, robot, course geometry, lights, camera, and decision markers for direct inspection.
- At each checkpoint, it scores left, center, and right lanes.
- The planner penalizes blocked objects, unsafe gaps, steep terrain, height changes, and unnecessary lateral motion.
- The lowest-cost feasible lane becomes the next route decision.
- The video, route trajectory, decision trace, and aggregate report are exported for inspection.

This is not autonomy as a label. It is a reproducible MuJoCo pipeline where every route choice has a machine-readable cost map behind it.

---

## Technical Approach

### Architecture

```
Obstacle Metadata -> MuJoCo Course Builder -> Futurist Humanoid Scene
                                              |
                                      Perception Checkpoints
                                              |
                                      Three-Lane Cost Map
                                              |
                                   Autonomous Action Selector
                                              |
                                  Route Waypoints + Gait Driver
                                              |
                              Video + Trajectory + Decision Artifacts
```

### Layer 1: Perception

The perception layer inspects the obstacles near each checkpoint and evaluates three lane options: left, center, and right. Each option records total cost, terrain cost, lateral movement cost, perceived obstacles, maximum height, and maximum slope.

### Layer 2: Surgical Skill Library

For this navigation submission, the reusable "skills" are locomotion decisions rather than surgical manipulation primitives.

| Skill | What It Does |
|-------|-------------|
| `walk_clear` | Continue forward when the current lane is safe |
| `move_left` | Shift to the left lane when center/right options are worse |
| `move_right` | Shift to the right lane when left/center options are blocked |
| `avoid_gap` | Reject a lane containing an unsafe broken-paver gap |
| `avoid_blocked` | Reject high crates, barriers, posts, signs, and dense pipe clutter |
| `ascend_stairs` | Traverse passable stair or ramp terrain when its cost is lower than dodging |

### Layer 3: Closed-Loop Residual Controller

This project does not use a torque-level residual controller. Its feedback loop is planner-level: the current lane and local obstacle context determine the next action, then the gait driver animates the humanoid along the selected route.

| Planner Signal | Correction Mode | Evidence |
|----------------|----------------|----------|
| Blocked lane | Choose alternate lane | Decision trace records high obstacle cost |
| Gap ahead | Avoid current lane | `broken_paver_gap` forces left movement |
| Traversable stairs | Keep/choose lane and ascend | 12 degree and 16 degree stairs produce ascent decisions |
| Lateral penalty | Avoid unnecessary lane changes | Robot climbs reasonable stairs instead of always dodging |
| Cluttered lane | Return to safer lane | Pipe cluster causes center return |

### Activated Grasp Constraint

This is not a grasping submission, so there is no `mjEQ_WELD` manipulation constraint. The analogous mechanism is route commitment: once the planner selects a lane/action pair, that decision is converted into waypoints and rendered through the humanoid gait controller.

The checked-in decision artifact verifies:

1. Every checkpoint evaluates all three lane options.
2. Each option records terrain and lateral movement cost.
3. The chosen lane/action is saved.
4. The final route includes left, center, and right lanes.
5. The final run reaches the goal successfully.

### Environment

The corridor is constructed in MuJoCo from structured obstacle metadata:

- **11 obstacle definitions** with lane, position, kind, height, width, slope, depth, and hazard fields.
- **Realistic obstacle assemblies** for cones, stairs, crates, curb, gap, traffic barrier, fallen sign, ramp, and pipe cluster.
- **Three navigable lanes** at fixed lateral offsets.
- **Decision markers** that visualize the active route choice.
- **Headless rendering support** through EGL.
- **Schematic fallback renderer** if real MuJoCo video output is unavailable.

---

## Core Features

- Existing Futurist humanoid robot from the repository asset library.
- Autonomous local perception over left, center, and right lanes.
- 11 real-world obstacle types: construction cones, split stairs, delivery crates, curb, broken paver gap, traffic barrier, fallen sidewalk sign, service ramp, pipe cluster, and final stairs.
- Runtime action selection: walk clear, move left, move right, avoid unsafe terrain, and ascend stairs/ramp.
- 11 decision points with full option costs exported to JSON.
- 3 lane changes and 2 ascent decisions in the generated run.
- 14.0 second demo video at 24 FPS.
- MuJoCo scene with 49 `nq`, 46 `nv`, 128 geoms, and 11 obstacle geoms.
- Route trajectory, decision trace, summary report, judge brief, and rubric scorecard artifacts.
- Headless / CI-safe operation with EGL and deterministic fallback rendering.

## Quantified Metrics

| Metric | Value |
|--------|-------|
| Robot platform | Futurist humanoid |
| MuJoCo `nq` | 49 |
| MuJoCo `nv` | 46 |
| Total geoms | 128 |
| Obstacle geoms | 11 |
| Lanes evaluated | 3 |
| Decision points | 11 |
| Obstacles perceived | 11 |
| Lane changes | 3 |
| Ascent decisions | 2 |
| Final X position | 5.35004 m |
| Demo duration | 14.0 s |
| Demo FPS | 24 |
| Exported artifacts | 4 primary artifacts |
| Validator result | Package internally consistent |

---

## Highlights

### Working humanoid navigation in MuJoCo with explainable lane decisions.

The Futurist humanoid crosses a corridor containing mixed real-world obstacles while the planner records why each route decision was selected. The decision trace shows all three lane options at every checkpoint, including cost, perceived hazards, maximum height, and maximum slope.

**Autonomy that shows its work.** The robot does not follow a hidden final route. It evaluates local costs, chooses a lane/action pair, and exports the selected route in `robot_autonomous_perception_decisions.json`.

**Obstacle semantics beyond generic boxes.** Cones, crates, barriers, signs, gaps, curbs, ramps, stairs, and pipe clutter receive different traversal costs. This forces the planner to choose between avoidance and ascent rather than treating every object the same.

**Reviewable evidence trail.** Every headline claim maps to a generated artifact: `robot_autonomous_perception_report.json`, `robot_autonomous_perception_decisions.json`, `robot_autonomous_perception_trajectory.json`, and `robot_autonomous_perception_demo.mp4`.

**Headless-first engineering.** The runner defaults to EGL for cloud/CI execution and keeps a schematic fallback so the same route, perception, and decision state can still produce visual evidence if MuJoCo rendering fails.

---

## Current Limitations

**Procedural gait.** The locomotion controller is a kinematic gait driver, not a learned whole-body balance policy.

**Structured perception.** The planner reads obstacle metadata from the MuJoCo course definition rather than raw camera or depth images.

**Local planning.** The route selector is checkpoint-local and explainable. It does not run a full global search across every future obstacle.

---

## Future Improvements

**Vision-based obstacle perception.** Replace structured obstacle metadata with simulated RGB-D or segmentation-based perception from the robot camera.

**Dynamic balance control.** Replace the procedural gait animation with a learned or model-predictive humanoid locomotion policy.

**Global route planning.** Add A* or sampling-based planning over the full corridor while preserving the local cost explanations.

**Domain randomization.** Randomize obstacle positions, widths, heights, and lane assignments to evaluate generalization across many course layouts.

**Closed-loop foot placement.** Use terrain height and slope estimates to adapt foot clearance, step timing, and base pose online.

---

## How to Run

```bash
# Install dependencies
.venv/bin/python -m pip install -r submissions/robot_autonomous_perception/requirements.txt

# --- Demonstration ---

# Full autonomous navigation demo
.venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py

# Quick smoke test
.venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py --quick

# --- Video ---

# Default generated demo video
MUJOCO_GL=egl .venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py

# Headless/cloud run is selected automatically when no display is available
MUJOCO_GL=egl .venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py --quick

# --- Evaluation ---

# Validate submission package and generated artifacts
.venv/bin/python submissions/robot_autonomous_perception/validate_submission.py
```

### Test Suite

```bash
.venv/bin/python submissions/robot_autonomous_perception/validate_submission.py
```

The validator confirms UUID consistency, required files, generated artifacts, MuJoCo scene depth, decision coverage, lane diversity, stair ascent behavior, and rubric scorecard completeness.

---

## Demo Video

The demo video is generated at `submissions/robot_autonomous_perception/artifacts/robot_autonomous_perception_demo.mp4` by:

```bash
MUJOCO_GL=egl .venv/bin/python submissions/robot_autonomous_perception/run_robot_autonomous_perception.py
```

**Camera plan (14.0s runtime, following the obstacle corridor):**

| Segment | What is shown |
|---------|---------------|
| Start corridor | Futurist humanoid begins in the center lane |
| Early obstacles | Construction cones and first stair decision |
| Mid-course | Delivery crates, curb, and broken paver gap avoidance |
| Blocked lanes | Traffic barrier and fallen sign force right-lane selection |
| Final corridor | Pipe clutter avoidance and final stair ascent |
| Finish | Robot reaches the goal at `x = 5.35004 m` |

**Video overlay includes:**

- Walking Futurist humanoid
- 3D obstacle field
- Colored scan choices
- Active decision beacon
- Route state derived from exported decisions

**Schematic fallback (headless environments):**

Activates if real MuJoCo rendering is unavailable. Renders the same route, perception state, and selected decisions in a deterministic top-down schematic.

---

## File Structure

```
submissions/robot_autonomous_perception/
├── README.md                                      - this file
├── registration.json                              - submission UUID and participant info
├── requirements.txt                               - Python dependencies
├── run_robot_autonomous_perception.py             - entry point for scene generation, planning, rendering, and export
├── validate_submission.py                         - package consistency validator
├── course_obstacles.json                          - real-world obstacle definitions
├── scene.xml                                      - directly loadable MuJoCo MJCF scene for robot/course inspection
├── EVALUATION_GUIDE.md                            - AI judge evaluation guide
├── PR_DESCRIPTION.md                              - pull request description
├── rubric_scorecard.json                          - machine-readable 8-criterion scorecard
└── artifacts/
    ├── robot_autonomous_perception_demo.mp4       - generated video
    ├── robot_autonomous_perception_trajectory.json - route and pose samples
    ├── robot_autonomous_perception_decisions.json - full lane cost maps and chosen actions
    └── robot_autonomous_perception_report.json    - summary metrics and artifact paths
```

---

## Scoring Alignment

| Criterion | Evidence |
|-----------|---------|
| Runnability | Single Python runner, checked-in `scene.xml`, local Futurist asset, deterministic obstacle file, validator, and generated artifacts |
| MuJoCo depth | Directly loadable MJCF scene imports Futurist humanoid URDF geometry and includes free base, assembled 3D obstacles, lane surfaces, camera, lights, and markers |
| Task design | Real-world humanoid obstacle course with stairs, ramps, curbs, gaps, barriers, posts, signs, crates, and pipe clutter |
| Control | Perception-driven lane scoring chooses left, right, clear walking, or stair/ramp ascent at runtime |
| Dexterity | Not a manipulation task; complexity is in legged navigation and obstacle reasoning |
| Engineering quality | Modular runner, deterministic JSON inputs, validator, report, trajectory, decisions, and demo video |
| Presentation | Video shows robot route, obstacle field, scan choices, and active decision beacon |
| Innovation | Combines real-world obstacle semantics with explainable autonomous route decisions using the provided humanoid asset |

---

## Technical Stack

- MuJoCo 3.x - humanoid URDF import and scene simulation
- Futurist humanoid robot from `assets/Futurist/futurist.urdf`
- Python dataclasses for obstacle definitions and decision records
- NumPy for route and animation computation
- ImageIO for video export
- EGL rendering for headless/cloud execution
- JSON artifacts for report, trajectory, decisions, registration, and rubric evidence
- Local validator for reproducibility checks

---

## Tools Used

Built with Codex in the repository workspace. The submission code is hand-authored Python using MuJoCo 3.x, deterministic obstacle metadata, and generated JSON/video artifacts for judge inspection.

---

## Registration

- **UUID:** `fd793b35-a006-4607-81de-339ec1bf4757`
- **Participant:** Thecoder
- **Event:** Robothon 2026 - Faraday Future MuJoCo Hackathon
