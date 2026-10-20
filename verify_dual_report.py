"""验收已完成的双机器人报告；失败返回非零退出码。"""
import argparse
from collections import Counter
import json
from pathlib import Path


def verify(report):
    errors = []
    def check(condition, message):
        if not condition:
            errors.append(message)
    tasks = report.get("tasks", [])
    robots = report.get("robots", [])
    events = report.get("events", [])
    ids = [t["task_id"] for t in tasks]
    check(report.get("completed") is True, "simulation not completed")
    check(len(robots) == 2 and len({r["robot_id"] for r in robots}) == 2, "expected two unique robots")
    check(len(tasks) >= 4, "acceptance demo requires at least four tasks")
    check(len(ids) == len(set(ids)), "duplicate task IDs")
    check(report.get("completed_tasks") == len(tasks), "completed task count mismatch")
    check(report.get("task_count") == len(tasks), "task count mismatch")
    check(report.get("simultaneous_motion_seconds", 0) > 0, "no simultaneous motion recorded")
    check(report.get("robot_overlap_frames") == 0, "robot overlap recorded")
    distance = report.get("min_robot_center_distance_m")
    check(distance is not None and distance > report.get("robot_contact_threshold_m", .56),
          "robot minimum distance is unsafe or missing")
    check(all(r.get("completed_tasks", 0) > 0 for r in robots), "both robots must complete tasks")
    for event_type in ("task_created", "task_assigned", "task_arrived", "task_completed"):
        counts = Counter(e.get("task_id") for e in events if e.get("event") == event_type)
        check(counts == Counter(ids), f"missing or duplicate {event_type}")
    robot_ids = {r["robot_id"] for r in robots}
    for task in tasks:
        check(task.get("status") == "completed", f"{task['task_id']} incomplete")
        check(task.get("robot_id") in robot_ids, f"{task['task_id']} invalid owner")
        times = [task.get(k) for k in ("created_at", "assigned_at", "completed_at")]
        check(all(isinstance(t, (float, int)) for t in times) and times == sorted(times, key=lambda t: -1 if t is None else t),
              f"{task['task_id']} invalid timestamps")
        for event in events:
            if event.get("task_id") == task["task_id"] and "robot_id" in event:
                check(event["robot_id"] == task["robot_id"], f"{task['task_id']} inconsistent owner")
    return errors


def main():
    parser = argparse.ArgumentParser(description="Validate dual robot completion report")
    parser.add_argument("report", nargs="?", default="demo_outputs/dual_robot_report.json")
    args = parser.parse_args()
    try:
        report = json.loads(Path(args.report).read_text(encoding="utf-8"))
        errors = verify(report)
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(f"FAIL: cannot validate report: {error}")
        return 1
    if errors:
        for error in errors:
            print("FAIL: " + error)
        return 1
    print(f"PASS: {report['completed_tasks']}/{report['task_count']} tasks; "
          f"simulated {report['simulated_seconds']}s; "
          f"simultaneous motion {report['simultaneous_motion_seconds']}s; "
          f"minimum robot distance {report['min_robot_center_distance_m']}m")
    print("Report checks passed. Run test_dual_robot_demo for per-frame collision checks.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
