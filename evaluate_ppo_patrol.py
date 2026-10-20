"""固定相同种子比较 PPO、A*+一步安全检查、随机策略，保存原始逐回合结果。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from stable_baselines3 import PPO
import torch

from ppo_patrol_env import PatrolPPOEnv


def run_episode(env, controller, seed, model=None, trace=None):
    observation, _ = env.reset(seed=seed)
    random = np.random.default_rng(seed)
    start = time.perf_counter()
    action_counts = [0]*9
    while True:
        if controller == "ppo":
            action = int(model.predict(observation, deterministic=True)[0])
        elif controller == "astar":
            action = env.baseline_action()
        else:
            action = int(random.integers(0,9))
        observation, _, terminated, truncated, info = env.step(action)
        action_counts[action] += 1
        if trace is not None:
            trace.append(dict(step=env.steps, action=action, x=float(env.xy[0]), y=float(env.xy[1]),
                              person_x=float(env.person[0]), person_y=float(env.person[1]),
                              outcome=info["outcome"]))
        if terminated or truncated:
            break
    return dict(controller=controller, mode=env.mode, seed=seed,
                wall_seconds=time.perf_counter()-start, action_counts=action_counts, **info)


def evaluate(model_path, episodes=20, first_seed=2000):
    torch.set_num_threads(1)
    model = PPO.load(model_path, device="cpu")
    rows = []
    for mode in ("static", "dynamic"):
        env = PatrolPPOEnv(mode)
        try:
            for controller in ("ppo", "astar", "random"):
                for seed in range(first_seed,first_seed+episodes):
                    rows.append(run_episode(env,controller,seed,model))
        finally:
            env.close()
    summary = []
    for mode in ("static", "dynamic"):
        for controller in ("ppo", "astar", "random"):
            group = [r for r in rows if r["mode"]==mode and r["controller"]==controller]
            successes = [r for r in group if r["success"]]
            summary.append(dict(mode=mode, controller=controller, episodes=len(group),
                                successes=len(successes), success_rate=len(successes)/len(group),
                                collision_rate=sum(r["collision"] for r in group)/len(group),
                                timeout_rate=sum(r["outcome"]=="timeout" for r in group)/len(group),
                                mean_episode_seconds=float(np.mean([r["simulated_seconds"] for r in group])),
                                mean_success_seconds=float(np.mean([r["simulated_seconds"] for r in successes])) if successes else None,
                                mean_success_distance_m=float(np.mean([r["distance_m"] for r in successes])) if successes else None))
    return dict(env_version=PatrolPPOEnv.VERSION, first_seed=first_seed,
                episodes_per_mode=episodes, deterministic_ppo=True,
                model_sha256=hashlib.sha256(Path(model_path).read_bytes()).hexdigest(),
                summary=summary, episodes=rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="ppo_artifacts/ppo_patrol.zip")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=2000)
    parser.add_argument("--output", default="ppo_artifacts/evaluation.json")
    args = parser.parse_args()
    if args.episodes <= 0:
        parser.error("episodes must be positive")
    result = evaluate(args.model,args.episodes,args.seed)
    path = Path(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2),encoding="utf-8")
    with path.with_suffix(".csv").open("w",newline="",encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream,fieldnames=list(result["episodes"][0]))
        writer.writeheader()
        writer.writerows(result["episodes"])
    for row in result["summary"]:
        print(json.dumps(row))
    lines=["# PPO 第二阶段评估结果", "", "同一模式下三种控制器使用相同场景种子；PPO 使用确定性推理。",
           "碰撞率包含被几何检查判定为碰撞的动作尝试，没有安全动作替换。", "",
           "| 场景 | 控制器 | 回合数 | 成功率 | 碰撞率 | 超时率 | 成功回合平均仿真秒 |",
           "| --- | --- | --- | --- | --- | --- | --- |"]
    for row in result["summary"]:
        seconds=f"{row['mean_success_seconds']:.2f}" if row['mean_success_seconds'] is not None else "—"
        lines.append(f"| {row['mode']} | {row['controller']} | {row['episodes']} | {row['success_rate']:.0%} | "
                     f"{row['collision_rate']:.0%} | {row['timeout_rate']:.0%} | {seconds} |")
    lines += ["", f"评估种子：{args.seed} 至 {args.seed+args.episodes-1}。",
              f"模型 SHA256：`{result['model_sha256']}`。", "",
              "这是小型仿真环境的实验记录，不代表医院实用性能。成功回合耗时有幸存者偏差，",
              "必须结合成功率和碰撞率阅读，不能仅凭较短耗时认定策略更好。",
              "PPO 使用 0.6 秒风险观测，A* 对照检查下一步 0.2 秒，两者都知道人员运动模型。",
              "这是理想状态下的系统配置比较，不是相同预测窗口的纯算法消融，也不是现实部署比较。"]
    path.with_suffix(".md").write_text("\n".join(lines)+"\n",encoding="utf-8")


if __name__ == "__main__":
    main()
