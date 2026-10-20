"""在相同种子下评估双机器人 PPO 与原版 A*，保存原始数据。"""
import argparse
import csv
import json
from pathlib import Path
import time
import torch
from dual_ppo_demo import load_policy, run_episode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="dual_ppo_artifacts/ppo_dual.zip")
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=3000)
    parser.add_argument("--seconds", type=float, default=180)
    parser.add_argument("--scenarios", nargs="+", choices=["corridor", "ring"], default=["corridor", "ring"])
    parser.add_argument("--output", default="dual_ppo_artifacts/evaluation.json")
    args = parser.parse_args()
    if args.episodes < 1:
        parser.error("--episodes must be positive")
    torch.set_num_threads(1)
    model, sha = load_policy(args.model)
    rows = []
    for scenario in args.scenarios:
        for people in (False, True):
            for controller in ("ppo", "astar"):
                for seed in range(args.seed, args.seed+args.episodes):
                    started = time.perf_counter()
                    report = run_episode(controller, model, scenario, seed, people, args.seconds)
                    row = dict(scenario=scenario, people=people, controller=controller, seed=seed,
                               outcome=report["outcome"], completed_tasks=report["completed_tasks"],
                               task_count=report["task_count"], seconds=report["simulated_seconds"],
                               distance_m=sum(r["distance_m"] for r in report["robots"]),
                               rejected_frames=sum(report["safety_rejected_frames"]),
                               overlap_frames=report["robot_overlap_frames"],
                               min_robot_distance=report["min_robot_center_distance_m"],
                               wall_seconds=time.perf_counter()-started)
                    rows.append(row)
                    if controller == "ppo":
                        row["action_histogram_r1"] = [sum(t["actions"][0] == a for t in report["trajectory"]) for a in range(9)]
                        row["action_histogram_r2"] = [sum(t["actions"][1] == a for t in report["trajectory"]) for a in range(9)]
                    else:
                        row["action_histogram_r1"] = None
                        row["action_histogram_r2"] = None
                    print(f"{scenario} people={people} {controller} seed={seed}: {row['outcome']} {row['seconds']}s", flush=True)
    summary = []
    for scenario in args.scenarios:
        for people in (False, True):
            for controller in ("ppo", "astar"):
                group = [r for r in rows if (r["scenario"],r["people"],r["controller"]) == (scenario,people,controller)]
                successes = [r for r in group if r["outcome"] == "success"]
                summary.append(dict(scenario=scenario, people=people, controller=controller,
                    episodes=len(group), successes=len(successes), success_rate=len(successes)/len(group),
                    mean_success_seconds=sum(r["seconds"] for r in successes)/len(successes) if successes else None,
                    safety_rejected_frames=sum(r["rejected_frames"] for r in group),
                    robot_overlap_frames=sum(r["overlap_frames"] for r in group)))
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(model_sha256=sha, model=args.model, env_version="ward-dual-joint-v1",
                                  seed_start=args.seed, max_seconds=args.seconds, summary=summary, episodes=rows),
                               ensure_ascii=False, indent=2), encoding="utf-8")
    with path.with_suffix(".csv").open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# 第三阶段实测对照", "", f"模型 SHA256：`{sha}`", "",
             "| 场景 | 人员 | 控制器 | 成功/回合 | 成功平均秒 | 安全拦截帧 | 机器人重叠帧 |",
             "| --- | --- | --- | --- | --- | --- | --- |"]
    for s in summary:
        mean = s["mean_success_seconds"]
        lines.append(f"| {s['scenario']} | {s['people']} | {s['controller']} | {s['successes']}/{s['episodes']} | "
                     f"{f'{mean:.2f}' if mean is not None else '无成功回合'} | {s['safety_rejected_frames']} | {s['robot_overlap_frames']} |")
    lines += ["", "安全拦截为动作被安全闸停车的仿真帧数，不是物理碰撞次数。", "环形为跨布局测试；训练只覆盖配置中记录的布局。"]
    path.with_suffix(".md").write_text("\n".join(lines)+"\n", encoding="utf-8")


if __name__ == "__main__":
    main()
