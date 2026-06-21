## Adaptive Hand Lab

Registration UUID: fd793b35-a006-4607-81de-339ec1bf4757

### Project Summary

Adaptive Hand Lab is a self-contained MuJoCo dexterity submission for FFAI Robothon 2026. A five-finger gantry-mounted hand scans a sterile workspace, approaches and grasps a medication vial, rotates its cap in-hand, transports the vial to a sterile pod, inserts it, presses an audit button, recovers from a deterministic slip disturbance, and exports video and structured controller evidence.

### Key Innovations

- **Five-finger manipulation:** Twenty independently actuated finger joints coordinate thumb opposition, a balanced vial grasp, in-hand cap rotation, transport, insertion, and partial release.
- **Long-horizon medication workflow:** Eight phases cover workspace scan, visual-servo approach, adaptive grasp, cap rotation, transport, pod insertion, audit, and slip recovery.
- **Sensor-rich MuJoCo model:** 25 actuators, 37 generalized velocities, 11 frame-position/joint-position/touch sensors, free-joint task objects, and 26 geoms.
- **Residual control:** A deterministic minimum-jerk stage planner applies bounded visual-servo corrections while contact-balance and slip-observer signals track grasp stability and recovery.
- **Robustness evidence:** Forty fixed-seed perturbation cases spanning initial pose, cap friction, and slip impulses compare the residual controller with its baseline.
- **Headless rendering:** A deterministic telemetry-rich schematic video is generated when MuJoCo offscreen rendering is unavailable.

### Recorded Results

- Runtime task success: 100%
- Workflow completion: 8/8 task results
- Peak active fingers: 5
- Stable five-finger samples: 227
- Final slip-observer error: 0.059 mm
- Median visual-servo error: 19.08 mm raw, 5.77 mm corrected
- Visual-servo error reduction: 69.76%
- Residual-policy success: 100% across 40 robustness cases
- Baseline success: 30%
- Median endpoint error: 65.056 mm baseline, 9.177 mm with residual control

### Run Instructions

From the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r submissions/adaptive_hand_lab/requirements.txt
.venv/bin/python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py
```

Quick smoke test:

```bash
.venv/bin/python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py --quick
```

Validate the completed package:

```bash
.venv/bin/python submissions/adaptive_hand_lab/validate_submission.py
```

### Submission Files

- `adaptive_hand_lab_scene.xml` — MuJoCo hand, sterile workcell, task objects, actuators, and sensors
- `run_adaptive_hand_lab.py` — Controller, simulation, rendering, artifact export, and robustness evaluation
- `validate_submission.py` — Submission package and metric consistency checks
- `requirements.txt` — Python dependencies
- `README.md` — Full project documentation and evidence disclosures
- `evaluation_report.json` — Rubric-aligned evidence index
- `registration.json` — Participant and registration metadata
- `artifacts/adaptive_hand_lab_demo.mp4` — Generated demonstration
- `artifacts/adaptive_hand_lab_trajectory.json` — Sampled actions, state, feedback, contact, and sensor values
- `artifacts/adaptive_hand_lab_report.json` — Runtime result, model statistics, task results, and aggregate metrics
- `artifacts/adaptive_hand_lab_eval.json` — Forty baseline/residual perturbation comparisons
- `artifacts/adaptive_hand_lab_contact_timeline.json` — Five-finger contact, balance, and slip-recovery timeline
- `artifacts/adaptive_hand_lab_policy_card.json` — Controller topology and closed-loop metrics
- `artifacts/adaptive_hand_lab_narration.srt` — Demonstration narration track
