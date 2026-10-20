"""自适应动态规划演示、统一初始条件及回合执行。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import numpy as np
import pybullet as p
from adaptive_patrol import AdaptiveSimulation
from dual_robot_demo import DualRobotSimulation, DT, RADIUS
from patrol_scenarios import initialize_episode


def load_q(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("version") != "adaptive-q-v1":
        raise ValueError("Expected adaptive-q-v1 policy")
    if data.get("completed") is not True:
        raise ValueError("Policy training is incomplete")
    q = np.asarray(data["q_values"], dtype=float)
    if q.shape != (36,4) or not np.isfinite(q).all():
        raise ValueError("Invalid Q table")
    return q


def create_sim(scenario, seed, people=True, controller="adaptive", gui=False, beds=None):
    cls = DualRobotSimulation if controller == "astar" else AdaptiveSimulation
    sim = cls(kind=scenario,gui=gui,people=people,beds=beds,verbose=False)
    try:
        sim.initial_conditions = initialize_episode(sim,seed,beds)
    except Exception:
        sim.close()
        raise
    sim.dispatch()
    if controller == "astar":
        for robot in sim.robots:
            task = sim.task_for(robot)
            goal = task.goal if task else robot.home
            if math.dist(robot.xy,goal) >= .025:
                sim.replan_robot(robot,goal)
    return sim


def run(scenario="corridor", seed=4000, people=True, controller="adaptive", q=None,
        seconds=180., gui=False, beds=None, learner=None, capture=True):
    started = time.perf_counter()
    sim = create_sim(scenario,seed,people,controller,gui,beds)
    trajectory = []
    interrupted = False
    static_collision_frames = person_collision_frames = 0
    min_person_clearance = math.inf
    try:
        while not sim.done and sim.elapsed+1e-9 < seconds:
            if gui:
                keys = p.getKeyboardEvents()
                if keys.get(27,0) & p.KEY_WAS_TRIGGERED:
                    interrupted = True
                    break
                if keys.get(32,0) & p.KEY_WAS_TRIGGERED:
                    sim.paused = not sim.paused
                if sim.paused:
                    time.sleep(.02)
                    continue
            states, modes = [], []
            before = [r.xy for r in sim.robots]
            goals = [(sim.task_for(r).goal if r.task_id else r.home) for r in sim.robots]
            completed = [r.completed_tasks for r in sim.robots]
            if controller != "astar":
                states = [sim.state(r) for r in sim.robots]
                if learner:
                    modes = learner.choose(states)
                elif controller == "adaptive":
                    if q is None:
                        raise ValueError("adaptive controller requires a trained Q table")
                    modes = [int(np.argmax(q[s])) for s in states]
                elif controller == "fixed":
                    modes = [1,1]
                elif controller == "short":
                    modes = [0,0]
                else:
                    raise ValueError("Unknown controller")
                sim.control(modes)
                reward_targets = [r.path[0] if r.path else goal for r,goal in zip(sim.robots,goals)]
            for _ in range(12):
                if sim.done or sim.elapsed+1e-9 >= seconds:
                    break
                sim.tick()
                static_collision_frames += int(any(not sim.free(r.xy,False,RADIUS) for r in sim.robots))
                gaps = [math.dist(r.xy,a.xy)-RADIUS-a.radius for r in sim.robots for a in sim.ward.actors]
                if gaps:
                    min_person_clearance = min(min_person_clearance,min(gaps))
                    person_collision_frames += int(min(gaps)<=0)
            if learner:
                next_states = [sim.state(r) for r in sim.robots]
                rewards = [4*(math.dist(start,goal)-math.dist(r.xy,goal))-.04
                           +15*(r.completed_tasks-count)
                           for r,start,goal,count in zip(sim.robots,before,reward_targets,completed)]
                learner.update(states,modes,rewards,next_states,sim.done or sim.elapsed+1e-9>=seconds)
            if capture:
                trajectory.append(dict(time=round(sim.elapsed,4),positions=[r.xy for r in sim.robots],
                                       states=states,modes=modes))
            if gui:
                time.sleep(.2)
        report = sim.report()
        report.update(controller=controller,seed=seed,people=people,
                      initial_conditions=sim.initial_conditions,
                      outcome="interrupted" if interrupted else "success" if sim.done else "timeout",
                      static_collision_frames=static_collision_frames,person_collision_frames=person_collision_frames,
                      min_person_clearance_m=min_person_clearance if math.isfinite(min_person_clearance) else None,
                      wall_seconds=time.perf_counter()-started,trajectory=trajectory)
        return report
    finally:
        sim.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario",choices=["corridor","ring"],default="corridor")
    parser.add_argument("--controller",choices=["adaptive","astar","fixed","short"],default="adaptive")
    parser.add_argument("--policy",default="adaptive_artifacts/q_policy.json")
    parser.add_argument("--seed",type=int,default=4000)
    parser.add_argument("--seconds",type=float,default=180)
    parser.add_argument("--no-people",action="store_true")
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--beds",nargs="+")
    parser.add_argument("--report",default="demo_outputs/adaptive_report.json")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:
        parser.error("--seconds must be finite and positive")
    try:
        q = load_q(args.policy) if args.controller=="adaptive" else None
        report = run(args.scenario,args.seed,not args.no_people,args.controller,q,args.seconds,not args.headless,args.beds)
    except (ValueError,FileNotFoundError) as error:
        parser.error(str(error))
    if q is not None:
        report["policy_sha256"] = hashlib.sha256(Path(args.policy).read_bytes()).hexdigest()
    path = Path(args.report)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(f"{report['outcome']}: {report['completed_tasks']}/{report['task_count']} in {report['simulated_seconds']}s")
    return 0 if report["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
