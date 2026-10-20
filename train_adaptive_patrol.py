"""在实际双机器人病区仿真中训练共享 Q-learning 规划模式选择器。"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from adaptive_patrol_demo import run


class QLearner:
    def __init__(self,seed=23):
        self.q = np.zeros((36,4))
        self.visits = np.zeros((36,4),dtype=int)
        self.rng = np.random.default_rng(seed)
        self.epsilon = .4
        self.transitions = 0
        self.reward = 0.

    def choose(self,states):
        actions = []
        for state in states:
            values = self.q[state]
            if self.rng.random() < self.epsilon:
                action = int(self.rng.integers(4))
            else:
                ties = np.flatnonzero(values >= values.max()-1e-10)
                action = int(self.rng.choice(ties))
            actions.append(action)
        return actions

    def update(self,states,actions,rewards,next_states,done):
        for s,a,r,n in zip(states,actions,rewards,next_states):
            target = r+(0. if done else .97*self.q[n].max())
            self.q[s,a] += .12*(target-self.q[s,a])
            self.visits[s,a] += 1
            self.transitions += 1
            self.reward += r


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes",type=int,default=60)
    parser.add_argument("--seed",type=int,default=23)
    parser.add_argument("--scenario-seed",type=int,default=1000)
    parser.add_argument("--output",default="adaptive_artifacts")
    args = parser.parse_args()
    if args.episodes<4:
        parser.error("--episodes must be >= 4")
    output = Path(args.output)
    if output.exists() and any(output.iterdir()):
        parser.error("Output directory must be new or empty")
    output.mkdir(parents=True,exist_ok=True)
    learner = QLearner(args.seed)
    rows = []
    started = time.perf_counter()
    completed = False
    try:
        for episode in range(args.episodes):
            learner.epsilon = .4*(1-episode/max(args.episodes-1,1))+.05
            learner.reward = 0.
            scenario = "corridor" if episode%2 == 0 else "ring"
            report = run(scenario,args.scenario_seed+episode,people=True,controller="adaptive",
                         learner=learner,capture=False,seconds=120)
            row = dict(episode=episode+1,scenario=scenario,seed=args.scenario_seed+episode,
                       success=report["completed"],seconds=report["simulated_seconds"],
                       reward=learner.reward,epsilon=learner.epsilon,
                       transitions=learner.transitions)
            rows.append(row)
            print(f"Episode {episode+1}/{args.episodes} {scenario}: {report['outcome']} {row['seconds']}s",flush=True)
        completed = True
    finally:
        config = dict(version="adaptive-q-v1",algorithm="tabular off-policy Q-learning",
                      completed=completed,seed=args.seed,scenario_seed_start=args.scenario_seed,
                      requested_episodes=args.episodes,actual_episodes=len(rows),
                      transitions=learner.transitions,alpha=.12,gamma=.97,
                      epsilon_start=.45,epsilon_end=.05,max_episode_seconds=120,
                      states=36,actions=["short","cruise","detour","yield"],
                      wall_seconds=time.perf_counter()-started,
                      q_values=learner.q.tolist(),visits=learner.visits.tolist())
        (output/"q_policy.json").write_text(json.dumps(config,indent=2),encoding="utf-8")
        (output/"training.json").write_text(json.dumps(rows,indent=2),encoding="utf-8")


if __name__ == "__main__":
    main()
