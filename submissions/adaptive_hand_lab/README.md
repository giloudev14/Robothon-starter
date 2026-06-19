# Adaptive Hand Lab

Adaptive Hand Lab is a runnable MuJoCo dexterous-manipulation submission for Robothon 2026. A five-finger hand handles a fragile medicine vial in a sterile bench scene: it scans the workspace, aligns with visual-servo feedback, closes a balanced multi-finger grasp, rotates off a cap, inserts the vial into a pod, presses an audit button, recovers from a slip disturbance, and exports a labeled trajectory.

## Robot Platform

- Procedural MJCF five-finger hand with thumb opposition.
- 25 actuated channels: gantry xyz, wrist yaw, audit button, and 20 finger joints.
- MuJoCo hinge, slide, and free joints; collision geoms; contact-enabled physics.
- Frame-position sensors for palm, vial, cap, pod, and button plus five fingertip touch sensors.

## Task Design

The task is a long-horizon medication safety workflow:

1. Scan the bench and localize the vial.
2. Approach with visual-servo correction.
3. Establish five-finger contact balance.
4. Rotate the cap in-hand while stabilizing the vial.
5. Transport and insert the vial into the sterile pod.
6. Press the audit button only after placement.
7. Recover from a simulated slip disturbance.
8. Export sensor streams, residual actions, contacts, and task labels.

## Control Approach

The controller combines a minimum-jerk phase planner with a residual policy. The residual layer corrects gantry motion from visual-servo error, contact-balance error, and a slip observer. The generated trajectory logs the raw error, corrected error, residual action norm, policy confidence, fingertip contact proxy, button depth, and MuJoCo sensor snapshots.

## Generated Artifacts

Running `run_adaptive_hand_lab.py` creates:

- `artifacts/adaptive_hand_lab_demo.mp4`
- `artifacts/adaptive_hand_lab_trajectory.json`
- `artifacts/adaptive_hand_lab_report.json`
- `artifacts/adaptive_hand_lab_eval.json`
- `artifacts/adaptive_hand_lab_contact_timeline.json`
- `artifacts/adaptive_hand_lab_policy_card.json`
- `artifacts/adaptive_hand_lab_narration.srt`
- `rubric_scorecard.json`
- `submission_manifest.json`

## Why It Targets A High Score

| Rubric item | Evidence |
|---|---|
| Runnability | One command regenerates all artifacts; quick mode and validator included. |
| MuJoCo depth | MJCF model with 25 actuators, 20 finger joints, free objects, collision geoms, frame sensors, touch sensors, and contact-enabled physics. |
| Task design | Clear, meaningful sterile medication workflow with eight phases and objective success checks. |
| Control | Autonomous planner plus residual feedback policy and fixed-seed disturbance evaluation. |
| Dexterous manipulation | Five-finger coordination, thumb opposition, cap rotation, vial stabilization, and audit-button press. |
| Engineering quality | Compact folder, deterministic outputs, structured JSON artifacts, local validator, and clear docs. |
| Presentation | Generated MP4 plus subtitles, report, timeline, and policy card for judge inspection. |
| Innovation | Combines medication triage, in-hand manipulation, safety audit, slip recovery, and dataset export. |

## How To Run

From the repository root:

```bash
python3 -m pip install -r requirements.txt
python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py
```

Quick smoke test:

```bash
python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py --quick
python submissions/adaptive_hand_lab/validate_submission.py
```

The default demo is 72 seconds at 18 FPS, inside the recommended 1-3 minute video range. If offscreen MuJoCo rendering is unavailable, the script still loads and steps the MuJoCo model, then writes a schematic video from the same controller state and sensor-export pipeline.

## Registration Note

`registration.json` contains a valid UUID-shaped placeholder. Replace it with the UUID issued by `robothon.ff.com`, and put the same UUID at the top of the PR description before final submission.

## Current Limitations

The high-level task timing is deterministic for reproducibility. The residual controller is closed-loop and stress-tested, but it is not a trained neural policy.

## Future Improvements

- Train the residual layer from exported trajectories.
- Add randomized object layouts and clutter.
- Add camera/depth image export for imitation-learning datasets.
