"""从本次训练CSV和评估JSON生成汇报图，不填充虚构数据。"""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", default="dual_ppo_artifacts")
    args = parser.parse_args()
    root = Path(args.directory)
    evaluation = json.loads((root/"evaluation.json").read_text(encoding="utf-8"))
    with (root/"monitor.csv").open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(line for line in handle if not line.startswith("#")))
    fig, axes = plt.subplots(2,2,figsize=(12,8))
    axes[0,0].plot(np.arange(1,len(rows)+1), [float(r["r"]) for r in rows], color="#377eb8")
    axes[0,0].set(title="Training episode return", xlabel="Episode", ylabel="Team return")
    groups = [(s,p) for s in ("corridor","ring") for p in (False,True)]
    x = np.arange(len(groups))
    for controller, offset, color in (("ppo",-.18,"#377eb8"),("astar",.18,"#ff7f00")):
        summaries = [next(r for r in evaluation["summary"] if (r["scenario"],r["people"],r["controller"]) == (s,p,controller))
                     for s,p in groups]
        axes[0,1].bar(x+offset,[r["success_rate"]*100 for r in summaries],.36,label=controller,color=color)
        axes[1,0].bar(x+offset,[r["mean_success_seconds"] if r["mean_success_seconds"] is not None else np.nan
                               for r in summaries],.36,label=controller,color=color)
    labels = [f"{s}\n{'people' if p else 'static'}" for s,p in groups]
    for ax in (axes[0,1],axes[1,0]):
        ax.set_xticks(x,labels)
        ax.legend()
    axes[0,1].set(title="Batch completion rate (with safety layer)",ylabel="Percent",ylim=(0,110))
    axes[1,0].set(title="Completion time: successful episodes only",ylabel="Simulation seconds")
    for index,color in ((1,"#e41a1c"),(2,"#377eb8")):
        counts = np.sum([r[f"action_histogram_r{index}"] for r in evaluation["episodes"] if r["controller"] == "ppo"],axis=0)
        axes[1,1].bar(np.arange(9)+(index-1.5)*.36,counts,.36,label=f"R{index}",color=color)
    axes[1,1].set(title="Deterministic PPO action counts",xlabel="0=wait, 1=forward, 2..8=turn",ylabel="Control cycles",xticks=range(9))
    axes[1,1].legend()
    for ax in axes.flat:
        ax.grid(axis="y",alpha=.2)
    fig.suptitle("Stage 3: joint PPO + A* + safety stop | held-out seeds")
    fig.tight_layout()
    fig.savefig(root/"results.png",dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
