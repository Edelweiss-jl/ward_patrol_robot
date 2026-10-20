"""加载已训练的 PPO，独立运行单机器人 A* 路点导航演示。"""
import argparse
import json
from pathlib import Path
import time

import pybullet as p
from stable_baselines3 import PPO
import torch

from ppo_patrol_env import PatrolPPOEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model",default="ppo_artifacts/ppo_patrol.zip")
    parser.add_argument("--controller",choices=("ppo","astar"),default="ppo")
    parser.add_argument("--mode",choices=("static","dynamic"),default="dynamic")
    parser.add_argument("--seed",type=int,default=1000)
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--report",default="demo_outputs/ppo_demo_report.json")
    parser.add_argument("--screenshot",help="Save initial scene PNG")
    args = parser.parse_args()
    torch.set_num_threads(1)
    model = PPO.load(args.model,device="cpu") if args.controller=="ppo" else None
    env = PatrolPPOEnv(args.mode,"rgb_array" if args.headless else "human")
    trace=[]
    result=dict(outcome="interrupted",success=False,controller=args.controller,seed=args.seed,mode=args.mode)
    try:
        obs,_=env.reset(seed=args.seed)
        if args.screenshot:
            from PIL import Image
            original=env.render_mode
            env.render_mode="rgb_array"
            path=Path(args.screenshot)
            path.parent.mkdir(parents=True,exist_ok=True)
            Image.fromarray(env.render()).save(path)
            env.render_mode=original
        print(f"Controller={args.controller}; seed={args.seed}; SPACE pause, ESC exit")
        paused=False
        while p.isConnected(env.client):
            if not args.headless:
                keys=p.getKeyboardEvents(physicsClientId=env.client)
                if keys.get(27,0)&p.KEY_WAS_TRIGGERED:
                    break
                if keys.get(32,0)&p.KEY_WAS_TRIGGERED:
                    paused=not paused
                if paused:
                    time.sleep(.05)
                    continue
            action=int(model.predict(obs,deterministic=True)[0]) if model else env.baseline_action()
            obs,reward,terminated,truncated,info=env.step(action)
            trace.append(dict(step=env.steps,action=action,x=float(env.xy[0]),y=float(env.xy[1]),reward=reward))
            if terminated or truncated:
                result.update(info)
                print(json.dumps(result,indent=2))
                if not args.headless:
                    print("Episode finished. Press ESC to close; rerun with another --seed for another episode.")
                    while p.isConnected(env.client):
                        if p.getKeyboardEvents(physicsClientId=env.client).get(27,0)&p.KEY_WAS_TRIGGERED:
                            break
                        time.sleep(.05)
                break
            if not args.headless:
                time.sleep(env.DT)
    except KeyboardInterrupt:
        pass
    finally:
        result["trajectory"]=trace
        path=Path(args.report)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,indent=2),encoding="utf-8")
        env.close()
    return 0 if result["success"] else 1


if __name__=="__main__":
    raise SystemExit(main())
