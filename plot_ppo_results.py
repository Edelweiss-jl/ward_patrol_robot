"""从真实训练与评估记录生成可用于汇报的图，不参与训练。"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--directory",default="ppo_artifacts")
    args=parser.parse_args()
    folder=Path(args.directory)
    result=json.loads((folder/"evaluation.json").read_text(encoding="utf-8"))
    rows=result["summary"]
    fig,axes=plt.subplots(1,2,figsize=(13,4.6),layout="constrained")
    x=np.arange(len(rows))
    bottom=np.zeros(len(rows))
    for key,label,color in (("success_rate","Success","#228b76"),
                            ("collision_rate","Collision attempt","#ce5353"),
                            ("timeout_rate","Timeout","#d4a34c")):
        values=np.array([r[key] for r in rows])*100
        axes[0].bar(x,values,bottom=bottom,label=label,color=color)
        bottom+=values
    axes[0].set_xticks(x,[f"{r['mode']}\n{r['controller']}" for r in rows],fontsize=9)
    axes[0].set_ylim(0,110)
    axes[0].set_ylabel("Episodes (%)")
    axes[0].set_title(f"Held-out evaluation ({result['episodes_per_mode']} episodes / group)")
    axes[0].legend(fontsize=8,loc="upper center",bbox_to_anchor=(.5,-.17),ncol=3,frameon=False)
    static=pd.read_csv(folder/"static_monitor.csv",comment="#")
    dynamic=pd.read_csv(folder/"dynamic_monitor.csv",comment="#")
    rewards=pd.concat([static["r"],dynamic["r"]],ignore_index=True)
    axes[1].plot(rewards,alpha=.15,color="#487baf",linewidth=.7)
    axes[1].plot(rewards.rolling(30,min_periods=1).mean(),color="#215b91",label="30-episode mean")
    axes[1].axvline(len(static),color="#ae7635",linestyle="--",label="Dynamic training starts")
    axes[1].set_xlabel("Completed training episode")
    axes[1].set_ylabel("Episode return")
    axes[1].set_title("Training return (not success rate)")
    axes[1].legend(fontsize=8)
    for axis in axes:
        axis.spines[["top","right"]].set_visible(False)
    fig.savefig(folder/"results.png",dpi=180)
    plt.close(fig)
    print(folder/"results.png")


if __name__=="__main__":
    main()
