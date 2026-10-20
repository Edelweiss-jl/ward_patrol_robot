"""自适应动态导航：Q-learning选择局部规划模式，连续几何安全闸。"""
from __future__ import annotations
import math
import time
import numpy as np
import pybullet as p
from dual_robot_demo import DualRobotSimulation, DT, RADIUS, MARGIN


class AdaptiveSimulation(DualRobotSimulation):
    MODES = ("short", "cruise", "detour", "yield")
    LOOKAHEAD = (.45, 1.4, 2.0, .45)

    def __init__(self, *args, **kwargs):
        self.modes = [1, 1]
        self.local_targets = [None, None]
        self.headings = [None, None]
        self.last_motion = [0., 0.]
        self.last_goals = [None, None]
        self.last_plans = [-100., -100.]
        self.plan_calls = 0
        self.planning_wall_seconds = 0.
        self.rejected_frames = [0, 0]
        self.mode_counts = np.zeros((2,4), dtype=int)
        self.last_actor_xy = None
        self.actor_velocity = []
        self.trajectory = []
        super().__init__(*args, **kwargs)

    def route(self, robot, goal, dynamic=True):
        start = time.perf_counter()
        result = super().route(robot, goal, dynamic)
        self.planning_wall_seconds += time.perf_counter()-start
        self.plan_calls += 1
        return result

    def smooth(self, points):
        if not points:
            return []
        output, index = [points[0]], 0
        while index < len(points)-1:
            nxt = len(points)-1
            while nxt > index+1 and not self.segment(points[index], points[nxt], False):
                nxt -= 1
            output.append(points[nxt])
            index = nxt
        return output

    def make_plan(self, robot, goal):
        i = self.robots.index(robot)
        # 全局静态骨架不会因某个人恰好占住目标而整条路径消失。
        points = self.route(robot, goal, dynamic=False)
        robot.path = self.smooth(points)
        robot.replans += 1
        self.last_goals[i] = goal
        self.last_plans[i] = self.elapsed
        self.emit("path_planned", robot, self.task_for(robot), reachable=bool(robot.path),
                  waypoint_count=len(robot.path), planner="static A* + visibility smoothing")
        self.draw_path(robot)

    def target(self, robot, lookahead):
        points = [np.asarray(robot.xy)] + [np.asarray(x) for x in robot.path]
        while len(robot.path) > 1 and math.dist(robot.xy, robot.path[0]) < .12:
            robot.path.pop(0)
        points = [np.asarray(robot.xy)] + [np.asarray(x) for x in robot.path]
        if len(points) < 2:
            return np.asarray(robot.xy)
        # 先跳过可直视的旧折点；再在当前线段取前视目标。
        while len(robot.path) > 1 and self.segment(robot.xy, robot.path[1], False):
            robot.path.pop(0)
        delta = np.asarray(robot.path[0])-robot.xy
        length = np.linalg.norm(delta)
        return np.asarray(robot.xy)+delta*min(1.,lookahead/max(length,1e-9))

    def state(self, robot):
        i = self.robots.index(robot)
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        nearby = min([math.dist(robot.xy,a.xy)-RADIUS-a.radius for a in self.ward.actors]+[10.])
        other = math.dist(robot.xy,self.robots[1-i].xy)
        # 3x3x2x2=36种离散状态，无床号、种子或测试结果泄漏。
        obstacle_bin = 0 if nearby > 1.5 else 1 if nearby > .6 else 2
        peer_bin = 0 if other > 3 else 1 if other > 1.2 else 2
        stuck = int(self.elapsed-self.last_motion[i] > 1.)
        near_goal = int(math.dist(robot.xy, goal) < 1.2)
        return ((obstacle_bin*3+peer_bin)*2+stuck)*2+near_goal

    def decide(self, robot, mode):
        i = self.robots.index(robot)
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        if math.dist(robot.xy,goal) < .025:
            self.headings[i] = None
            return
        lost_visibility = robot.path and not self.segment(robot.xy,robot.path[0],False)
        if goal != self.last_goals[i] or lost_visibility or (not robot.path and self.elapsed-self.last_plans[i] > 1.):
            self.make_plan(robot, goal)
        self.mode_counts[i, mode] += 1
        if mode == 3 or not robot.path or math.dist(robot.xy,goal) < .025:
            self.headings[i] = None
            return
        target = self.target(robot,self.LOOKAHEAD[mode])
        self.local_targets[i] = target
        vector = target-robot.xy
        angle = math.atan2(vector[1],vector[0])
        horizon = .65 if mode == 0 else 1.0
        options = []
        for degrees in (0, -20, 20, -40, 40, -65, 65, -90, 90, -125, 125, 180):
            direction = np.array([math.cos(angle+math.radians(degrees)),math.sin(angle+math.radians(degrees))])
            end = np.asarray(robot.xy)+direction*min(.95*horizon, np.linalg.norm(vector))
            if not self.safe_segment(robot,robot.xy,end):
                continue
            if not self.segment(end,robot.path[0],False):
                continue
            # 以最近一次实测速度做短期预测，不读取人员未来真实轨迹。
            clearance = 5.
            for actor, velocity in zip(self.ward.actors,self.actor_velocity):
                relative = np.asarray(robot.xy)-actor.xy
                delta = end-robot.xy-velocity*horizon
                fraction = np.clip(-np.dot(relative,delta)/max(np.dot(delta,delta),1e-9),0,1)
                clearance = min(clearance,float(np.linalg.norm(relative+fraction*delta))-RADIUS-actor.radius)
            if clearance < MARGIN:
                continue
            progress = np.linalg.norm(vector)-math.dist(end,target)
            score = progress-.12*abs(degrees)/180+.10*min(clearance,1.)
            if mode == 2:
                score += .2*min(clearance,1.)
            options.append((score,-abs(degrees),direction))
        self.headings[i] = max(options,key=lambda x:x[:2])[2] if options else None

    def advance_robot(self, robot):
        i = self.robots.index(robot)
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        if math.dist(robot.xy,goal) < .025:
            return super().advance_robot(robot)
        direction = self.headings[i]
        if direction is None:
            self.begin_wait(robot,"adaptive_wait")
            return False
        # 在局部目标近处避免越过；速度、半径、余量均与A*基线相同。
        length = min(.95*DT, math.dist(robot.xy,self.local_targets[i]))
        end = np.asarray(robot.xy)+direction*length
        if not self.safe_segment(robot,robot.xy,end):
            self.rejected_frames[i] += 1
            self.begin_wait(robot,"adaptive_safety_stop")
            return False
        self.end_wait(robot)
        robot.xy = tuple(float(x) for x in end)
        robot.distance += length
        robot.status = "moving" if task else "returning"
        self.last_motion[i] = self.elapsed
        yaw = math.atan2(direction[1],direction[0])
        p.resetBasePositionAndOrientation(robot.body,[*end,.245],p.getQuaternionFromEuler([0,0,yaw]))
        p.resetBasePositionAndOrientation(robot.marker,[end[0]+.22*math.cos(yaw),end[1]+.22*math.sin(yaw),.48],[0,0,0,1])
        return True

    def control(self, modes):
        now = [np.asarray(a.xy) for a in self.ward.actors]
        self.actor_velocity = ([np.zeros(2) for _ in now] if self.last_actor_xy is None else
                               [(a-b)/.2 for a,b in zip(now,self.last_actor_xy)])
        self.last_actor_xy = now
        self.modes = list(modes)
        for robot, mode in zip(self.robots,modes):
            self.decide(robot,int(mode))

    def refresh_ui(self):
        super().refresh_ui()
        count = sum(t.status == "completed" for t in self.tasks)
        self.status_id = p.addUserDebugText(
            f"ADAPTIVE Q | DONE {count}/{len(self.tasks)} | MODES {self.modes} | SPACE pause  ESC exit",
            [-self.ward.hx,-self.ward.hy-.7,.1],[.1,.2,.3],1.1,replaceItemUniqueId=self.status_id)

    def report(self):
        report = super().report()
        report.update(planner="adaptive-q-v1", mode_counts=self.mode_counts.tolist(),
                      global_plan_calls=self.plan_calls, global_planning_wall_seconds=self.planning_wall_seconds,
                      rejected_frames=self.rejected_frames, trajectory=self.trajectory)
        report["navigation_debug"] = dict(paths=[r.path for r in self.robots],
            local_targets=[x.tolist() if x is not None else None for x in self.local_targets],
            headings=[x.tolist() if x is not None else None for x in self.headings],
            actors=[a.xy for a in self.ward.actors])
        return report
