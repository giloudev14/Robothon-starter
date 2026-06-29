from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parent
UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$", re.I)


def fail(message: str) -> int:
    print(f"[fail] {message}")
    return 1


def main() -> int:
    registration = json.loads((ROOT / "registration.json").read_text(encoding="utf-8"))
    uuid = registration.get("uuid", "")
    if not UUID_RE.match(uuid):
        return fail("registration.json must contain a valid UUID string.")
    pr_text = (ROOT / "PR_DESCRIPTION.md").read_text(encoding="utf-8")
    if uuid not in pr_text:
        return fail("PR_DESCRIPTION.md UUID must match registration.json.")

    required = [
        ROOT / "README.md",
        ROOT / "EVALUATION_GUIDE.md",
        ROOT / "PR_DESCRIPTION.md",
        ROOT / "rubric_scorecard.json",
        ROOT / "course_obstacles.json",
        ROOT / "scene.xml",
        ROOT / "run_robot_autonomous_perception.py",
        ROOT / "requirements.txt",
        ROOT / "artifacts" / "robot_autonomous_perception_demo.mp4",
        ROOT / "artifacts" / "robot_autonomous_perception_trajectory.json",
        ROOT / "artifacts" / "robot_autonomous_perception_decisions.json",
        ROOT / "artifacts" / "robot_autonomous_perception_report.json",
    ]
    for path in required:
        if not path.exists():
            return fail(f"missing {path.relative_to(ROOT)}")
        if path.stat().st_size == 0:
            return fail(f"empty {path.relative_to(ROOT)}")

    report = json.loads((ROOT / "artifacts" / "robot_autonomous_perception_report.json").read_text(encoding="utf-8"))
    if not report.get("success"):
        return fail("report success must be true.")
    if "Futurist humanoid" not in report.get("robot_platform", ""):
        return fail("report must use the existing Futurist humanoid model.")
    if report["mujoco_depth"]["nq"] < 40 or report["mujoco_depth"]["ngeom"] < 100:
        return fail("expected the humanoid model plus a rich 3D obstacle scene.")
    try:
        scene_model = mujoco.MjModel.from_xml_path(str(ROOT / "scene.xml"))
    except Exception as exc:
        return fail(f"scene.xml must load directly in MuJoCo: {exc}")
    if scene_model.nq < 40 or scene_model.ngeom < 100:
        return fail("scene.xml must contain the humanoid and rich obstacle course.")
    if "scene.xml" not in report.get("scene_xml", "scene.xml"):
        return fail("report must reference the checked-in scene.xml.")
    metrics = report["autonomy_metrics"]
    if metrics["decision_points"] < 8:
        return fail("expected at least 8 autonomous decision points.")
    if metrics["lane_changes"] < 3:
        return fail("expected left/right avoidance decisions.")
    if metrics["ascend_decisions"] < 2:
        return fail("expected multiple stair/ramp ascent decisions.")
    if metrics["obstacles_perceived"] < 10:
        return fail("expected many real-world obstacles.")

    decisions = json.loads((ROOT / "artifacts" / "robot_autonomous_perception_decisions.json").read_text(encoding="utf-8"))["decisions"]
    chosen_lanes = {row["chosen_lane"] for row in decisions}
    actions = {row["action"] for row in decisions}
    if not {-1, 0, 1}.issubset(chosen_lanes):
        return fail("route must choose left, center, and right lanes.")
    if "ascend_stairs" not in actions:
        return fail("route must include a stair ascent decision.")

    scorecard = json.loads((ROOT / "rubric_scorecard.json").read_text(encoding="utf-8"))
    if len(scorecard.get("scorecard", {})) != 8:
        return fail("rubric_scorecard.json must cover all eight criteria.")

    print("[ok] Robot Autonomous Perception submission package is internally consistent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
