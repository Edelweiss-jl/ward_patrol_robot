"""训练真正的 SB3 PPO；输出模型、配置和训练 CSV。"""
import argparse
import json
from pathlib import Path
import time

import gymnasium
import numpy
import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
import torch

from ppo_patrol_env import PatrolPPOEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=40000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", default="ppo_artifacts")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if args.steps < 2048:
        parser.error("--steps must be at least 2048")
    output = Path(args.output)
    if (output / "ppo_patrol.zip").exists() and not args.overwrite:
        parser.error("Model exists: use a new --output directory or explicit --overwrite")
    output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(1)
    env = PatrolPPOEnv(mode="static")
    check_env(env, warn=True)
    env.close()
    env = Monitor(PatrolPPOEnv(mode="static"), str(output / "static_monitor.csv"))
    model = PPO("MlpPolicy", env, n_steps=512, batch_size=64, n_epochs=6,
                learning_rate=3e-4, gamma=.99, gae_lambda=.95, ent_coef=.015,
                policy_kwargs=dict(net_arch=[64,64]), seed=args.seed, device="cpu", verbose=0)
    model.set_logger(configure(str(output), ["csv"]))
    # 先学习沿路点移动，再在有移动人员的环境继续优化。
    warmup = max(1024, args.steps // 8)
    started = time.perf_counter()
    config = dict(env_version=PatrolPPOEnv.VERSION, algorithm="SB3 PPO", seed=args.seed,
                  observation_size=env.observation_space.shape[0],
                  requested_steps=args.steps, warmup_requested=warmup, device="cpu",
                  n_steps=512, batch_size=64, n_epochs=6, learning_rate=3e-4,
                  gamma=.99, gae_lambda=.95, ent_coef=.015, net_arch=[64,64],
                  versions=dict(torch=torch.__version__, numpy=numpy.__version__,
                                gymnasium=gymnasium.__version__, stable_baselines3=stable_baselines3.__version__))
    (output / "training_config.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
    try:
        print(f"Static warmup: {warmup} requested steps", flush=True)
        model.learn(warmup)
        config["static_actual_steps"] = model.num_timesteps
        model.save(output / "ppo_static")
        env.close()
        env = Monitor(PatrolPPOEnv(mode="dynamic"), str(output / "dynamic_monitor.csv"))
        env.reset(seed=args.seed+1)
        model.set_env(env)
        print(f"Dynamic training: {args.steps-warmup} requested steps", flush=True)
        model.learn(args.steps-warmup, reset_num_timesteps=False)
        model.save(output / "ppo_patrol")
        config.update(actual_steps=model.num_timesteps, training_wall_seconds=time.perf_counter()-started,
                      completed=True)
        print(f"Saved {output / 'ppo_patrol.zip'}; {model.num_timesteps} steps", flush=True)
    except KeyboardInterrupt:
        model.save(output / "ppo_interrupted")
        config.update(actual_steps=model.num_timesteps, completed=False)
        raise
    finally:
        (output / "training_config.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
        env.close()


if __name__ == "__main__":
    main()
