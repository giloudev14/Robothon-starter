from __future__ import annotations

import json
import re
import sys
from pathlib import Path


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
    if f"Registration UUID: {uuid}" not in pr_text:
        return fail("PR_DESCRIPTION.md UUID must match registration.json.")

    required = [
        ROOT / "adaptive_hand_lab_scene.xml",
        ROOT / "run_adaptive_hand_lab.py",
        ROOT / "README.md",
        ROOT / "JUDGE_BRIEF.md",
        ROOT / "rubric_scorecard.json",
        ROOT / "submission_manifest.json",
        ROOT / "artifacts" / "adaptive_hand_lab_demo.mp4",
        ROOT / "artifacts" / "adaptive_hand_lab_trajectory.json",
        ROOT / "artifacts" / "adaptive_hand_lab_report.json",
        ROOT / "artifacts" / "adaptive_hand_lab_eval.json",
        ROOT / "artifacts" / "adaptive_hand_lab_contact_timeline.json",
        ROOT / "artifacts" / "adaptive_hand_lab_policy_card.json",
        ROOT / "artifacts" / "adaptive_hand_lab_narration.srt",
    ]
    for path in required:
        if not path.exists():
            return fail(f"missing {path.relative_to(ROOT)}")
        if path.stat().st_size == 0:
            return fail(f"empty {path.relative_to(ROOT)}")

    report = json.loads((ROOT / "artifacts" / "adaptive_hand_lab_report.json").read_text(encoding="utf-8"))
    if not report.get("success"):
        return fail("report success must be true.")
    if report["mujoco_depth"]["nu_actuators"] < 24:
        return fail("expected at least 24 actuators.")
    if report["mujoco_depth"]["nsensors"] < 10:
        return fail("expected rich sensor coverage.")
    metrics = report["closed_loop_metrics"]
    if metrics["median_visual_servo_error_m"] > 0.01:
        return fail("median visual-servo error should be below 1 cm.")
    if metrics["servo_error_reduction_pct"] < 60:
        return fail("residual policy should reduce raw error by at least 60 percent.")
    if metrics["stable_five_finger_samples"] < 20:
        return fail("contact timeline needs stable five-finger contact.")

    evaluation = json.loads((ROOT / "artifacts" / "adaptive_hand_lab_eval.json").read_text(encoding="utf-8"))
    if evaluation["rollout_count"] < 40:
        return fail("stress evaluation needs at least 40 rollouts.")
    if evaluation["summary"]["residual_success_rate"] < 0.95:
        return fail("residual stress success rate should be at least 95 percent.")

    timeline = json.loads((ROOT / "artifacts" / "adaptive_hand_lab_contact_timeline.json").read_text(encoding="utf-8"))
    if timeline["finger_order"] != ["thumb", "index", "middle", "ring", "little"]:
        return fail("contact timeline must list all five fingers.")
    if timeline["summary"]["max_active_fingers"] < 5:
        return fail("contact timeline must show all five fingers active.")
    if timeline["summary"]["recovery_window_samples"] < 5:
        return fail("contact timeline must include recovery samples.")

    scorecard = json.loads((ROOT / "rubric_scorecard.json").read_text(encoding="utf-8"))
    if len(scorecard.get("scorecard", {})) != 8:
        return fail("rubric_scorecard.json must cover all eight criteria.")

    print("[ok] Adaptive Hand Lab submission package is internally consistent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
