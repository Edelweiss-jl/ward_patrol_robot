"""第三阶段：复杂病区中的双机器人联合 PPO 环境。"""
from __future__ import annotations

import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p

from dual_robot_demo import DualRobotSimulation, DT, RADIUS
from patrol_scenarios import initialize_episode


class PPODualSimulation(DualRobotSimulation):
    """保留原调度和生命周期，只替换运动动作。安全闸只能停车。"""

    def __init__(self, *args, **kwargs):
        self.actions = np.ones(2, dtype=int)
        self.blocked_frames = [0, 0]
        self.policy_wait_frames = [0, 0]
        self.progress_reward = 0.
        self.controller_name = "PPO"
        super().__init__(*args, **kwargs)

    def prepare(self, robot):
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        if math.dist(robot.xy, goal) < .025:
            return
        blocked = not robot.path or any(not self.safe_segment(robot, pt, pt)
                                        for pt in robot.path[:6])
        if blocked and robot.plan_timer >= .65:
            self.replan_robot(robot, goal)
        while robot.path and math.dist(robot.xy, robot.path[0]) < 1e-8:
            robot.path.pop(0)

    def delta(self, robot, action, duration=DT):
        if action == 0 or not robot.path:
            return np.zeros(2)
        vector = np.asarray(robot.path[0]) - robot.xy
        angle = math.atan2(vector[1], vector[0]) + (int(action)-1)*math.pi/4
        length = min(.95*duration, float(np.linalg.norm(vector)))
        return length*np.array([math.cos(angle), math.sin(angle)])

    def advance_robot(self, robot):
        index = self.robots.index(robot)
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        if math.dist(robot.xy, goal) < .025:
            return super().advance_robot(robot)
        robot.plan_timer += DT
        self.prepare(robot)
        if not robot.path:
            self.begin_wait(robot, "no_safe_path")
            return False
        action = int(self.actions[index])
        if action == 0:
            self.policy_wait_frames[index] += 1
            self.begin_wait(robot, "ppo_wait")
            return False
        start = np.array(robot.xy)
        end = start + self.delta(robot, action)
        if not self.safe_segment(robot, start, end):
            self.blocked_frames[index] += 1
            self.begin_wait(robot, "safety_rejected")
            return False
        self.end_wait(robot)
        # 路点距离减少值；偏离路点或反向运动会产生负奖励。
        self.progress_reward += math.dist(start, robot.path[0])-math.dist(end, robot.path[0])
        robot.xy = tuple(float(x) for x in end)
        robot.distance += float(np.linalg.norm(end-start))
        robot.status = "moving" if task else "returning"
        yaw = math.atan2(end[1]-start[1], end[0]-start[0])
        p.resetBasePositionAndOrientation(robot.body, [*end, .245], p.getQuaternionFromEuler([0,0,yaw]))
        p.resetBasePositionAndOrientation(robot.marker,
                                         [end[0]+.22*math.cos(yaw), end[1]+.22*math.sin(yaw), .48],
                                         [0,0,0,1])
        return True

    def refresh_ui(self):
        super().refresh_ui()
        state = "PAUSED" if self.paused else "COMPLETE" if self.done else "RUNNING"
        count = sum(t.status == "completed" for t in self.tasks)
        text = (f"{self.controller_name} DUAL | {state} | DONE {count}/{len(self.tasks)} | "
                f"ACTIONS {list(self.actions)} | REJECTED {sum(self.blocked_frames)} | SPACE: pause  ESC: exit")
        self.status_id = p.addUserDebugText(text, [-self.ward.hx, -self.ward.hy-.7, .1],
                                          [.1,.2,.3], 1.1, replaceItemUniqueId=self.status_id)

    def report(self):
        report = super().report()
        report.update(env_version=DualPatrolPPOEnv.VERSION,
                      safety_rejected_frames=self.blocked_frames,
                      policy_wait_frames=self.policy_wait_frames,
                      safety_layer="continuous segment rejection; stop only; no replacement action")
        return report


class DualPatrolPPOEnv(gym.Env):
    """单个联合策略输出 [R1动作,R2动作]；不是 MAPPO 或独立多智能体训练。"""
    VERSION = "ward-dual-joint-v1"
    CONTROL_DT = .2
    metadata = {"render_modes": ["human"], "render_fps": 5}

    def __init__(self, scenario="corridor", people=True, render_mode=None,
                 max_seconds=180., beds=None, randomize=True):
        if scenario not in ("corridor", "ring"):
            raise ValueError("Unknown scenario")
        if not math.isfinite(max_seconds) or max_seconds <= 0:
            raise ValueError("max_seconds must be finite and positive")
        self.scenario, self.people, self.render_mode = scenario, people, render_mode
        self.max_seconds, self.beds, self.randomize = max_seconds, beds, randomize
        self.action_space = spaces.MultiDiscrete([9, 9])
        self.observation_space = spaces.Box(-1, 1, (52,), dtype=np.float32)
        self.sim = None
        self.finished = True

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.close()
        self.sim = PPODualSimulation(self.scenario, self.render_mode == "human",
                                     beds=self.beds, people=self.people, verbose=False)
        if self.randomize:
            initialize_episode(self.sim,beds=self.beds,rng=self.np_random)
        self.sim.dispatch()
        for robot in self.sim.robots:
            self.sim.prepare(robot)
        self.steps = 0
        self.velocities = np.zeros((2, 2))
        self.episode_return = 0.
        self.finished = False
        return self.observe(), {"env_version": self.VERSION}

    def observe(self):
        result = []
        sim = self.sim
        for i, robot in enumerate(sim.robots):
            other = sim.robots[1-i]
            task = sim.task_for(robot)
            goal = task.goal if task else robot.home
            direction = np.asarray(robot.path[0] if robot.path else goal)-robot.xy
            direction /= max(float(np.linalg.norm(direction)), 1e-8)
            actor = min(sim.ward.actors, key=lambda a: math.dist(a.xy, robot.xy), default=None)
            rel, vel = np.zeros(2), np.zeros(2)
            if actor:
                rel = (np.asarray(actor.xy)-robot.xy)/20
                vector = np.asarray(actor.route[actor.target])-actor.xy
                vel = vector/max(float(np.linalg.norm(vector)), 1e-8)*actor.speed
            risks = [1. if sim.safe_segment(robot, robot.xy,
                       np.asarray(robot.xy)+sim.delta(robot, a, self.CONTROL_DT)) else -1.
                     for a in range(9)]
            # 26维/台：方向2、目标2、位置2、另一台相对位置2、
            # 另一台上一周期速度2、双方任务标志2、最近人员相对位置2/速度2、人员标志1、风险9。
            result.extend(np.r_[direction, (np.asarray(goal)-robot.xy)/20,
                np.asarray(robot.xy)/[sim.ward.hx, sim.ward.hy],
                (np.asarray(other.xy)-robot.xy)/20, self.velocities[1-i] if hasattr(self,"velocities") else [0,0],
                float(task is not None), float(other.task_id is not None), rel, vel, float(actor is not None), risks])
        return np.clip(result, -1, 1).astype(np.float32)

    def step(self, action):
        if self.finished:
            raise RuntimeError("reset required before step")
        action = np.asarray(action)
        if not self.action_space.contains(action):
            raise ValueError("Expected two integer actions in 0..8")
        sim = self.sim
        sim.actions = action.astype(int)
        before = np.array([r.xy for r in sim.robots])
        completed = sum(t.status == "completed" for t in sim.tasks)
        blocked = sum(sim.blocked_frames)
        sim.progress_reward = 0.
        for _ in range(round(self.CONTROL_DT/DT)):
            if sim.elapsed + 1e-9 >= self.max_seconds or sim.done:
                break
            sim.tick()
        self.velocities = (np.array([r.xy for r in sim.robots])-before)/self.CONTROL_DT
        for robot in sim.robots:
            sim.prepare(robot)
        self.steps += 1
        truncated = sim.elapsed + 1e-9 >= self.max_seconds and not sim.done
        self.finished = sim.done or truncated
        rejects = sum(sim.blocked_frames)-blocked
        reward = (4*sim.progress_reward-.03-.03*rejects
                  +15*(sum(t.status == "completed" for t in sim.tasks)-completed))
        if truncated:
            reward -= 10
        if sim.done:
            reward += 20
        self.episode_return += reward
        info = dict(success=bool(sim.done), outcome="success" if sim.done else "timeout" if truncated else "running",
                    completed_tasks=sum(t.status == "completed" for t in sim.tasks),
                    simulated_seconds=sim.elapsed, safety_rejected_frames=sum(sim.blocked_frames),
                    robot_overlap_frames=sim.safety_violations, episode_return=self.episode_return)
        return self.observe(), float(reward), bool(sim.done), bool(truncated), info

    def close(self):
        if self.sim is not None:
            self.sim.close()
            self.sim = None
