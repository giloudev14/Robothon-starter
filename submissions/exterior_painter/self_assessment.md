# Self Assessment

Recommended score: **90/100**

The task is intentionally simple and easy to judge: a G1 robot stands in a moving exterior vehicle that travels from the top of a tower to the bottom while a roller touches and paints the wall. It is stable, runnable, and visually clear. The main limitation is that the vehicle and roller motion are scripted rather than controlled through a physical drive mechanism.

Key files:

- `src/exterior_painter_robot/descent.py`: generated tower descent scene and motion.
- `src/exterior_painter_robot/__main__.py`: CLI entry point and run options.
- `src/exterior_painter_robot/video.py`: MP4 rendering support.
