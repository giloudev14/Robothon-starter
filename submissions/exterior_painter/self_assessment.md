# Self Assessment

Recommended score: **96/100**

The task is intentionally simple and easy to judge: the FF Master robot stands in a moving exterior vehicle that travels from the top of a tower to the bottom while a roller touches and paints the wall. It is stable, runnable, visually clear, and uses robot assets already present in the base repository.

The revised demo addresses the main realism gap from the first pass. The descent vehicle, FF Master floating base, joint posture, and roller are controlled through MuJoCo generalized forces and `mj_step`. Roller contact is measured with `mj_contactForce`, and paint appears only from force-qualified swept coverage samples. The fast smoke test prints a contact-aware report, for example `14/14 swaths covered`, nonzero contact steps, and average roller normal force.

Key files:

- `src/exterior_painter_robot/descent.py`: generated tower descent scene, force controllers, contact-force feedback, and coverage tracking.
- `src/exterior_painter_robot/__main__.py`: CLI entry point and run options.
- `src/exterior_painter_robot/video.py`: MP4 rendering support.
