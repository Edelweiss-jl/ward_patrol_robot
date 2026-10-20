"""训练第三阶段联合 PPO，输出真实模型和训练日志。"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import torch
import gymnasium
import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback
from dual_ppo_env import DualPatrolPPOEnv


class TrainingProgress(BaseCallback):
    def _on_step(self):
        if self.num_timesteps % 512 == 0:
            print(f"PPO rollout steps: {self.num_timesteps}", flush=True)
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=16384)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--scenario", choices=["corridor", "ring"], default="corridor")
    parser.add_argument("--output", default="dual_ppo_artifacts")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.steps < 512:
        parser.error("--steps must be at least 512")
    output = Path(args.output)
    if output.exists() and any(output.iterdir()) and not args.overwrite:
        parser.error("Output directory is not empty; use a new --output or --overwrite")
    output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    raw = DualPatrolPPOEnv(args.scenario)
    try:
        check_env(raw, warn=True)
    finally:
        raw.close()
    env = Monitor(DualPatrolPPOEnv(args.scenario), str(output/"monitor.csv"))
    model = PPO("MlpPolicy", env, n_steps=512, batch_size=64, n_epochs=6,
                learning_rate=1e-4, ent_coef=.005, gamma=.99, seed=args.seed,
                policy_kwargs=dict(net_arch=[64,64]), device="cpu", verbose=0)
    # 显式导航先验：初始 logits 偏向动作1，随后所有参数由 PPO 更新。
    # 不是专家轨迹、行为克隆或运行时动作替换。
    with torch.no_grad():
        model.policy.action_net.bias.zero_()
        model.policy.action_net.bias[[1,10]] = 3.
    model.set_logger(configure(str(output), ["csv"]))
    config = dict(env_version=raw.VERSION, algorithm="SB3 centralized joint PPO",
                  scenario=args.scenario, people=True, seed=args.seed,
                  observation_size=52, action_space=[9,9], requested_steps=args.steps,
                  control_dt=.2, max_seconds=180, initial_forward_logit_bias=3.,
                  n_steps=512, batch_size=64, n_epochs=6, learning_rate=1e-4,
                  ent_coef=.005, gamma=.99, net_arch=[64,64], completed=False,
                  versions=dict(torch=torch.__version__, numpy=np.__version__,
                                gymnasium=gymnasium.__version__, stable_baselines3=stable_baselines3.__version__))
    started = time.perf_counter()
    try:
        print(f"Training joint PPO: {args.steps} steps, {args.scenario}", flush=True)
        model.learn(args.steps, callback=TrainingProgress())
        model.save(output/"ppo_dual")
        config["completed"] = True
    except KeyboardInterrupt:
        model.save(output/"ppo_interrupted")
        raise
    finally:
        config.update(actual_steps=model.num_timesteps, wall_seconds=time.perf_counter()-started)
        (output/"training_config.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
        env.close()
    print(json.dumps(config, indent=2), flush=True)


if __name__ == "__main__":
    main()
