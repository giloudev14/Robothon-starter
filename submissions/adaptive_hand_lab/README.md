# Adaptive Hand Lab

**FFAI Robothon 2026 — Freestyle Category**

Adaptive Hand Lab is a self-contained MuJoCo dexterity submission in which a five-finger gantry-mounted hand scans a sterile workspace, approaches and grasps a medication vial, rotates its cap in-hand, transports the vial to a sterile pod, inserts it, presses an audit button, and recovers from a deterministic slip disturbance.

The project combines an eight-phase smooth stage planner with deterministic visual-servo, contact-balance, and slip-observer residuals. One command generates a demonstration video and machine-readable trajectory, contact, policy, evaluation, and runtime evidence.

## Project Overview

- **Five-finger platform:** 25 position-actuated channels control a three-axis gantry, wrist yaw, 20 finger joints, and the audit button.
- **Medication-handling task:** The hand performs a long-horizon scan, grasp, cap-rotation, transport, insertion, audit, and recovery workflow.
- **Closed-loop residual controller:** Reproducible visual-servo, contact-balance, and slip signals adjust the palm target and track grasp stability.
- **Structured evidence:** The runner exports sensor values, residual corrections, task phases, contact proxies, a five-finger contact timeline, and a 40-rollout robustness comparison.
- **Headless presentation:** If MuJoCo offscreen rendering is unavailable, the run produces a deterministic schematic video driven by the same controller state.

## Task Sequence

| # | Phase | Success signal |
|---:|---|---|
| 0 | Workspace scan | Palm, vial, cap, pod, button, and fingertip sensors are available |
| 1 | Visual-servo approach | The palm converges on the vial grasp pose |
| 2 | Adaptive grasp | All five fingers establish a balanced grasp |
| 3 | In-hand cap rotation | Thumb opposition and finger flexion maintain the vial while the cap rotates |
| 4 | Transport | The grasped vial moves from the pickup area toward the sterile pod |
| 5 | Pod insertion | The vial reaches the pod target and grip is partially released |
| 6 | Audit press | Button depth reaches the commanded completion position |
| 7 | Slip recovery and export | The modeled pose disturbance decays and the evidence artifacts are finalized |

## Technical Specifications

### Robot and MuJoCo model

- 25 actuators: gantry X/Y/Z, wrist yaw, 20 finger joints, and the audit-button slide.
- 11 sensors: five frame-position sensors, one button joint-position sensor, and five fingertip touch sensors.
- 37 generalized velocities and 26 geoms in the checked-in full run.
- Slide, hinge, and free joints with gravity, friction, damping, collision geometry, and position servos.
- Free-joint vial and cap bodies, a sterile pod and target, a medication bench, and a slide-actuated audit button.
- 2 ms MuJoCo timestep with a 960 × 544 default presentation frame.

### Controller

The deterministic policy has two layers:

1. An eight-phase smooth planner produces nominal palm, wrist, finger, object, and button targets.
2. A residual layer derives bounded XYZ corrections from visual-servo error while contact-balance and slip-observer signals track grasp and recovery quality.

Every sampled trajectory row records the active phase, raw and corrected visual-servo error, XYZ feedback correction, residual action norm, policy confidence, five analytical finger-contact proxies, contact balance, slip-observer error, button depth, and selected MuJoCo sensor outputs.

## Recorded Results

The checked-in full-run artifacts report:

| Metric | Result |
|---|---:|
| Runtime success | `true` |
| Task completion | 100% |
| Workflow phases | 8 |
| Peak active fingers | 5 |
| Stable five-finger samples | 227 |
| Final slip-observer error | 0.059 mm |
| Median raw visual-servo error | 19.08 mm |
| Median corrected visual-servo error | 5.77 mm |
| Servo-error reduction | 69.76% |
| Robustness rollouts | 40 |
| Residual-policy success | 100% |
| Baseline success | 30% |

These values come from `artifacts/adaptive_hand_lab_report.json` and `artifacts/adaptive_hand_lab_eval.json`. The rubric score in the report is an estimate, not an official competition result.

## Quick Start

Run from the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r submissions/adaptive_hand_lab/requirements.txt
.venv/bin/python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py
```

For a shorter smoke test:

```bash
.venv/bin/python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py --quick
```

The full run renders 72 seconds at 18 FPS. Quick mode renders 16 seconds at 12 FPS. Both modes write to `artifacts/adaptive_hand_lab_demo.mp4` and regenerate the JSON evidence files.

To check the completed submission package:

```bash
.venv/bin/python submissions/adaptive_hand_lab/validate_submission.py
```

## Submission Layout

```text
submissions/adaptive_hand_lab/
├── adaptive_hand_lab_scene.xml                 # MuJoCo workcell, robot, objects, sensors, and actuators
├── run_adaptive_hand_lab.py                    # Controller, simulation, rendering, and evaluation
├── validate_submission.py                      # Submission consistency checks
├── requirements.txt                            # Python dependencies
├── registration.json                           # Competition registration metadata
├── PR_DESCRIPTION.md                           # Pull-request summary
├── README.md                                   # Project documentation
└── artifacts/
    ├── adaptive_hand_lab_demo.mp4               # Generated full or quick demonstration
    ├── adaptive_hand_lab_trajectory.json        # Sampled control, state, metric, and sensor evidence
    ├── adaptive_hand_lab_report.json            # Runtime result and aggregate metrics
    ├── adaptive_hand_lab_eval.json              # 40 fixed-seed baseline/residual comparisons
    ├── adaptive_hand_lab_contact_timeline.json  # Sampled grip, contact-balance, and slip signals
    ├── adaptive_hand_lab_policy_card.json       # Controller topology and closed-loop metrics
    └── adaptive_hand_lab_narration.srt          # Demonstration narration track
```

The runner also generates `rubric_scorecard.json` and `submission_manifest.json` in the submission root.

## Evaluation Method

`artifacts/adaptive_hand_lab_eval.json` contains 40 deterministic perturbation cases spanning initial vial pose offsets, cap-friction variation, and slip impulses. Each case compares:

- **Baseline:** the disturbed rollout without the residual correction benefit.
- **Residual policy:** the rollout with visual-servo, contact, and slip feedback corrections.

In the checked-in sweep, median final error falls from 65.056 mm to 9.177 mm, with success increasing from 30% to 100% under the thresholds implemented in `stress_eval`.


## Why It Stands out?

1. It frames dexterous manipulation as a multi-stage sterile medication workflow rather than a conventional reach or pick-and-place demo.
2. Twenty finger joints coordinate thumb opposition, graded five-finger grasping, in-hand cap rotation, transport, insertion, and button interaction.
3. The evidence connects controller phases, residual corrections, sensor values, contact signals, slip recovery, and task progress.
4. The fixed-seed evaluation makes the residual controller's benefit directly comparable with its baseline.
5. The presentation remains reproducible in headless environments through a telemetry-rich schematic renderer.

## License

This project is submitted for the FFAI Robothon 2026 competition.

Registration UUID: `fd793b35-a006-4607-81de-339ec1bf4757`
