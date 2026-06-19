# Adaptive Hand Lab - Judge Brief

Registration UUID: f5b9c1c8-8f90-4fb9-9c7a-5e5f8a6b6c21

## Inspect First

1. `run_adaptive_hand_lab.py` - deterministic generator for video, trajectory, policy card, stress eval, contact timeline, and report.
2. `adaptive_hand_lab_scene.xml` - five-finger MJCF hand with 25 actuators, free objects, collisions, frame sensors, and touch sensors.
3. `artifacts/adaptive_hand_lab_report.json` - closed-loop and MuJoCo-depth metrics.
4. `artifacts/adaptive_hand_lab_eval.json` - 40 fixed-seed residual-vs-baseline disturbance tests.
5. `artifacts/adaptive_hand_lab_contact_timeline.json` - five-finger contact and slip recovery evidence.
6. `artifacts/adaptive_hand_lab_demo.mp4` - generated demonstration video.

## Quantitative Evidence

- Actuated channels: 25
- Hand topology: five-finger dexterous hand with thumb opposition
- Final task completion: 1.0
- Stress rollouts: 40
- Residual-policy success rate target: 1.0
- Median visual-servo error target: below 1 cm
- Residual controller logs raw error, corrected error, residual norm, confidence, contact balance, and slip observer

## Rubric Mapping

- Runnability: one command regenerates all evidence.
- MuJoCo depth: hinge/slide/free joints, collisions, sensors, actuators, contact-enabled physics.
- Task design: sterile medication triage is clear, meaningful, and long-horizon.
- Control: minimum-jerk planner plus residual visual-servo/contact/slip feedback.
- Dexterous manipulation: five-finger grasp, thumb opposition, cap rotation, vial transport, button press.
- Engineering quality: small package, deterministic seed, JSON artifacts, validator, clear docs.
- Presentation: generated MP4, subtitles, scorecard, timeline, and report.
- Innovation: safety-audited dexterous medication workflow with data export and recovery.
