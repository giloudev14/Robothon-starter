Registration UUID: f5b9c1c8-8f90-4fb9-9c7a-5e5f8a6b6c21

## Project Summary

- Project name: Adaptive Hand Lab
- Robot platform: Procedural MuJoCo five-finger dexterous hand with 25 actuated channels, thumb opposition, gantry wrist control, free-body vial/cap objects, touch sensors, frame sensors, collisions, and a slide-actuated audit button.
- Task goal: Complete a sterile medication handling workflow: scan, visual-servo to vial, five-finger grasp, in-hand cap rotation, transport, pod insertion, audit-button press, slip recovery, and trajectory export.
- Technical approach: Minimum-jerk stage planner with closed-loop residual corrections from visual-servo error, contact balance, and slip-observer signals.
- Core features: Multi-finger contact timeline, residual policy card, fixed-seed disturbance evaluation, generated MP4 demo, sensor-rich trajectory JSON, and rubric scorecard.
- Highlights: Designed to cover all eight scoring dimensions, especially MuJoCo depth, autonomous control, dexterous manipulation, engineering quality, presentation, and innovation.

## How To Run

```bash
python3 -m pip install -r requirements.txt
python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py
```

Quick smoke test:

```bash
python submissions/adaptive_hand_lab/run_adaptive_hand_lab.py --quick
python submissions/adaptive_hand_lab/validate_submission.py
```

## Demo Video

- Generated video: `submissions/adaptive_hand_lab/artifacts/adaptive_hand_lab_demo.mp4`
- Report: `submissions/adaptive_hand_lab/artifacts/adaptive_hand_lab_report.json`
- Evaluation: `submissions/adaptive_hand_lab/artifacts/adaptive_hand_lab_eval.json`
- Contact timeline: `submissions/adaptive_hand_lab/artifacts/adaptive_hand_lab_contact_timeline.json`
- Policy card: `submissions/adaptive_hand_lab/artifacts/adaptive_hand_lab_policy_card.json`
