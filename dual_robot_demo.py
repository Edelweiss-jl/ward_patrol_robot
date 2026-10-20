"""双机器人第一阶段：共享任务队列、A*、安全等待和事件日志。

只复用 complex_ward_demo 的场景、几何检查与 A*，不修改旧演示。
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
from pathlib import Path
import time

import pybullet as p

from complex_ward_demo import (
    Box, DT, MARGIN, RADIUS, Simulation, Ward, box_body, cylinder,
)


@dataclass
class PatrolTask:
    task_id: str
    bed_id: str
    goal: tuple
    created_at: float
    status: str = "pending"
    robot_id: str | None = None
    assigned_at: float | None = None
    completed_at: float | None = None


@dataclass
class RobotState:
    robot_id: str
    xy: tuple
    home: tuple
    body: int
    marker: int
    color: tuple
    task_id: str | None = None
    status: str = "idle"
    path: list = field(default_factory=list)
    dwell: float = 0.0
    plan_timer: float = 1.0
    distance: float = 0.0
    wait_seconds: float = 0.0
    replans: int = 0
    completed_tasks: int = 0
    waiting: bool = False
    lines: list = field(default_factory=list)
    label: int = -1


class DualRobotSimulation(Simulation):
    """一个 PyBullet 世界、两份机器人状态、一个单线程调度器。

    继承只用于复用场景搭建与规划；不调用旧版 tick / replan / report。
    与旧模块一致，一次进程只创建一个活动 Simulation。
    """

    def __init__(self, kind="corridor", gui=True, beds=None, people=True,
                 event_log=None, verbose=True):
        self.robots = []
        self.planning_robot = None
        ward = Ward(kind)
        if not people:
            ward.actors = []
        defaults = (["N1-1", "S3-2", "S1-1", "N3-2"] if kind == "corridor"
                    else ["ICU1", "WARD4", "WARD1", "ICU4"])
        selected = defaults if beds is None else list(beds)
        valid_beds = dict(ward.stops)
        if not selected or any(bed not in valid_beds for bed in selected):
            raise ValueError("beds must be nonempty and selected from: " + ", ".join(valid_beds))
        super().__init__(ward, gui)
        self.verbose = verbose
        self.log_handle = None
        self.tasks = []
        self.task_counter = 0
        self.schedule_timer = 1.0
        self.parallel_seconds = 0.0
        self.simultaneous_motion_seconds = 0.0
        self.min_robot_distance = math.inf
        self.safety_violations = 0
        self.finish_emitted = False
        self.ui_timer = 0.0
        self.selected_robot = 0
        self.blocked_target = None
        red, blue = (.9, .16, .18, 1), (.12, .42, .94, 1)
        home2 = (8, 0) if kind == "corridor" else (9.5, 0)
        self.robots = [
            RobotState("R1", ward.start, ward.start, self.robot, self.marker, red),
            RobotState("R2", home2, home2, cylinder(home2, RADIUS, .45, blue),
                       cylinder((home2[0] + .22, home2[1]), .07, .55,
                                (1, .9, .15, 1), False), blue),
        ]
        try:
            if event_log:
                path = Path(event_log)
                path.parent.mkdir(parents=True, exist_ok=True)
                self.log_handle = path.open("w", encoding="utf-8")
            self.emit("simulation_started", scenario=kind, robot_count=2)
            for bed in selected:
                self.add_task(bed)
            self.refresh_markers()
        except Exception:
            self.close()
            raise

    def emit(self, event, robot=None, task=None, **extra):
        item = dict(event_id=len(self.events) + 1, event=event,
                    sim_time=round(self.elapsed, 4))
        if robot is not None:
            item["robot_id"] = robot.robot_id
        if task is not None:
            item.update(task_id=task.task_id, bed_id=task.bed_id)
        item.update(extra)
        self.events.append(item)
        line = json.dumps(item, ensure_ascii=False)
        if self.verbose:
            print(line)
        if self.log_handle:
            self.log_handle.write(line + "\n")
            self.log_handle.flush()

    def add_task(self, bed_id):
        goals = dict(self.ward.stops)
        if bed_id not in goals:
            raise ValueError(f"Unknown bed: {bed_id}")
        self.task_counter += 1
        task = PatrolTask(f"T{self.task_counter:03d}", bed_id, goals[bed_id], self.elapsed)
        self.tasks.append(task)
        self.done = False
        self.finish_emitted = False
        self.schedule_timer = 1.0
        self.emit("task_created", task=task)
        return task

    def task_for(self, robot):
        return next((t for t in self.tasks if t.task_id == robot.task_id), None)

    def free(self, xy, dynamic=True, radius=RADIUS + MARGIN):
        if not super().free(xy, dynamic, radius):
            return False
        # 分配代价忽略瞬时人员/机器人；执行路径和每次移动都包含它们。
        return not dynamic or self.planning_robot is None or all(
            r is self.planning_robot or math.dist(xy, r.xy) > radius + RADIUS
            for r in self.robots)

    def route(self, robot, goal, dynamic=True):
        self.planning_robot = robot
        try:
            return self.plan(robot.xy, goal, dynamic)
        finally:
            self.planning_robot = None

    def segment(self, start, end, dynamic=True):
        """精确检测线段与膨胀矩形/圆，避免离散采样漏掉墙角。"""
        if not self.free(start, dynamic) or not self.free(end, dynamic):
            return False
        radius = RADIUS + MARGIN
        delta = (end[0] - start[0], end[1] - start[1])
        for box in self.ward.boxes + self.temp:
            lower = (box.x - box.sx / 2 - radius, box.y - box.sy / 2 - radius)
            upper = (box.x + box.sx / 2 + radius, box.y + box.sy / 2 + radius)
            entry, leave = 0.0, 1.0
            for axis in (0, 1):
                if abs(delta[axis]) < 1e-12:
                    if not lower[axis] <= start[axis] <= upper[axis]:
                        entry, leave = 1.0, 0.0
                        break
                else:
                    lo = (lower[axis] - start[axis]) / delta[axis]
                    hi = (upper[axis] - start[axis]) / delta[axis]
                    entry, leave = max(entry, min(lo, hi)), min(leave, max(lo, hi))
            if entry <= leave:
                return False
        if dynamic:
            circles = [(a.xy, radius + a.radius) for a in self.ward.actors]
            if self.planning_robot is not None:
                circles.extend((r.xy, radius + RADIUS) for r in self.robots
                               if r is not self.planning_robot)
            length2 = delta[0] ** 2 + delta[1] ** 2
            for center, clearance in circles:
                fraction = (sum((center[k] - start[k]) * delta[k] for k in (0, 1)) / length2
                            if length2 else 0.0)
                fraction = max(0.0, min(1.0, fraction))
                closest = tuple(start[k] + fraction * delta[k] for k in (0, 1))
                if math.dist(closest, center) <= clearance:
                    return False
        return True

    def safe_segment(self, robot, start, end):
        self.planning_robot = robot
        try:
            return self.segment(start, end)
        finally:
            self.planning_robot = None

    @staticmethod
    def path_length(path):
        return sum(math.dist(a, b) for a, b in zip(path, path[1:]))

    def dispatch(self):
        """FIFO 取任务，按 A* 路长选空闲机器人；不可达任务保留队列。

        这是贪心基线，不宣称全局最优。同一床位一次只允许一个活动任务。
        返回停车位中的机器人也可接受新任务。
        """
        idle = [r for r in self.robots if r.task_id is None]
        active_beds = {t.bed_id for t in self.tasks if t.status == "assigned"}
        for task in self.tasks:
            if not idle:
                break
            if task.status != "pending" or task.bed_id in active_beds:
                continue
            candidates = []
            for robot in idle:
                path = self.route(robot, task.goal, dynamic=False)
                if path:
                    candidates.append((self.path_length(path), robot.robot_id, robot))
            if not candidates:
                continue
            distance, _, robot = min(candidates, key=lambda c: (c[0], c[1]))
            task.status, task.robot_id, task.assigned_at = "assigned", robot.robot_id, self.elapsed
            robot.task_id, robot.dwell = task.task_id, 0.0
            robot.path = []
            robot.plan_timer = 1.0
            robot.status = "moving"
            self.end_wait(robot)
            idle.remove(robot)
            active_beds.add(task.bed_id)
            self.emit("task_assigned", robot, task, estimated_distance_m=round(distance, 3))

    def draw_path(self, robot):
        for line in robot.lines:
            p.removeUserDebugItem(line)
        robot.lines = []
        if self.gui:
            robot.lines = [p.addUserDebugLine([*a, .1], [*b, .1], robot.color[:3], 3)
                           for a, b in zip(robot.path, robot.path[1:])]

    def replan_robot(self, robot, goal):
        robot.path = self.route(robot, goal)
        robot.plan_timer = 0.0
        robot.replans += 1
        self.emit("path_planned", robot, self.task_for(robot),
                  reachable=bool(robot.path), waypoint_count=len(robot.path))
        self.draw_path(robot)

    def begin_wait(self, robot, reason):
        if not robot.waiting:
            robot.waiting = True
            self.emit("wait_started", robot, self.task_for(robot), reason=reason)
        robot.status = "waiting"
        robot.wait_seconds += DT

    def end_wait(self, robot):
        if robot.waiting:
            self.emit("wait_ended", robot, self.task_for(robot))
            robot.waiting = False

    def update_actors(self):
        # 沿用往返规则，但人员现在需要同时避让两台机器人。
        for actor in self.ward.actors:
            target = actor.route[actor.target]
            distance = math.dist(actor.xy, target)
            if distance < .015:
                actor.target = (actor.target + 1) % len(actor.route)
                continue
            step = min(actor.speed * DT, distance)
            nxt = tuple(actor.xy[k] + (target[k] - actor.xy[k]) * step / distance for k in (0, 1))
            # 若机器人进入人员的礼让余量，允许人员向远离机器人的方向退开。
            # 否则两个方向都被余量挡住，会永远原地折返。
            blocked = (any(math.dist(nxt, r.xy) < RADIUS + actor.radius + .02
                           or (math.dist(nxt, r.xy) < RADIUS + MARGIN + actor.radius + .04
                               and math.dist(nxt, r.xy) <= math.dist(actor.xy, r.xy))
                           for r in self.robots)
                       or any(other is not actor and math.dist(nxt, other.xy)
                              < actor.radius + other.radius + .08 for other in self.ward.actors)
                       or not self.free(nxt, False, actor.radius + .05))
            if blocked:
                actor.waiting += DT
                if actor.waiting >= 1:
                    actor.target = (actor.target + 1) % len(actor.route)
                    actor.waiting = 0
                continue
            actor.waiting = 0
            actor.xy = nxt
            p.resetBasePositionAndOrientation(actor.body, [*nxt, .45 if actor.cart else .77],
                                              [0, 0, 0, 1])

    def advance_robot(self, robot):
        task = self.task_for(robot)
        goal = task.goal if task else robot.home
        robot.plan_timer += DT
        if math.dist(robot.xy, goal) < .025:
            self.end_wait(robot)
            if task:
                if robot.dwell == 0:
                    self.emit("task_arrived", robot, task)
                robot.status = "dwelling"
                robot.dwell += DT
                if robot.dwell + 1e-9 >= .6:
                    task.status, task.completed_at = "completed", self.elapsed
                    robot.completed_tasks += 1
                    self.emit("task_completed", robot, task)
                    robot.task_id, robot.dwell = None, 0.0
                    robot.path = []
                    robot.status = "idle"
                    robot.plan_timer = 1.0
                    self.schedule_timer = 1.0
                    self.draw_path(robot)
            else:
                robot.status = "parked"
                if robot.path:
                    robot.path = []
                    self.draw_path(robot)
            return False
        blocked = not robot.path or any(
            not self.safe_segment(robot, pt, pt) for pt in robot.path[:6])
        if blocked and robot.plan_timer >= .65:
            self.replan_robot(robot, goal)
        while robot.path and math.dist(robot.xy, robot.path[0]) < 1e-9:
            robot.path.pop(0)
        if not robot.path:
            self.begin_wait(robot, "no_safe_path")
            return False
        target = robot.path[0]
        distance = math.dist(robot.xy, target)
        step = min(.95 * DT, distance)
        nxt = tuple(robot.xy[k] + (target[k] - robot.xy[k]) * step / distance for k in (0, 1))
        # 顺序提交：R2 检查时能看到本帧 R1 的新位置，禁止相互穿过/交换。
        if not self.safe_segment(robot, robot.xy, nxt):
            self.begin_wait(robot, "next_segment_blocked")
            if robot.plan_timer >= .65:
                self.replan_robot(robot, goal)
            return False
        self.end_wait(robot)
        robot.status = "moving" if task else "returning"
        yaw = math.atan2(nxt[1] - robot.xy[1], nxt[0] - robot.xy[0])
        robot.xy = nxt
        robot.distance += step
        p.resetBasePositionAndOrientation(robot.body, [*nxt, .245], p.getQuaternionFromEuler([0, 0, yaw]))
        p.resetBasePositionAndOrientation(robot.marker,
                                         [nxt[0] + .22 * math.cos(yaw), nxt[1] + .22 * math.sin(yaw), .48],
                                         [0, 0, 0, 1])
        return True

    def tick(self):
        if self.paused or self.done:
            return
        self.elapsed += DT
        self.schedule_timer += DT
        self.update_actors()
        if self.schedule_timer >= .65:
            self.dispatch()
            self.schedule_timer = 0.0
        if all(r.task_id is not None for r in self.robots):
            self.parallel_seconds += DT
        moved = [self.advance_robot(robot) for robot in self.robots]
        if all(moved):
            self.simultaneous_motion_seconds += DT
        distance = math.dist(self.robots[0].xy, self.robots[1].xy)
        self.min_robot_distance = min(self.min_robot_distance, distance)
        if distance <= 2 * RADIUS:
            self.safety_violations += 1
        p.stepSimulation()
        # 任务完成即结束；无需等待最后一次返航，避免把返航当成巡查耗时。
        self.done = all(t.status == "completed" for t in self.tasks)
        if self.done and not self.finish_emitted:
            self.finish_emitted = True
            self.emit("all_tasks_completed", task_count=len(self.tasks))
        self.ui_timer += DT
        if self.gui and self.ui_timer >= .1:
            self.refresh_ui()
            self.ui_timer = 0

    def refresh_markers(self):
        for (bed, _), body in zip(self.ward.stops, self.targets):
            tasks = [t for t in self.tasks if t.bed_id == bed]
            color = (.65, .65, .65, 1)
            if tasks:
                color = ((.2, .8, .35, 1) if all(t.status == "completed" for t in tasks)
                         else (1, .75, .1, 1))
            p.changeVisualShape(body, -1, rgbaColor=color)

    def refresh_ui(self):
        self.refresh_markers()
        for robot in self.robots:
            task = self.task_for(robot)
            text = f"{robot.robot_id} {robot.status.upper()} {task.task_id + ':' + task.bed_id if task else '-'}"
            robot.label = p.addUserDebugText(text, [*robot.xy, 1.1], robot.color[:3], 1.1,
                                             replaceItemUniqueId=robot.label)
        finished = sum(t.status == "completed" for t in self.tasks)
        pending = sum(t.status == "pending" for t in self.tasks)
        state = "PAUSED" if self.paused else "COMPLETE" if self.done else "RUNNING"
        text = (f"DUAL ROBOT | {state} | DONE {finished}/{len(self.tasks)} | QUEUED {pending} | "
                f"SELECT R{self.selected_robot + 1} | 1/2 B C R SPACE V L ESC")
        self.status_id = p.addUserDebugText(text, [-self.ward.hx, -self.ward.hy - .7, .1],
                                          [.1, .2, .3], 1.1, replaceItemUniqueId=self.status_id)

    def block_selected_target(self):
        """确定性验收按钮：在选中机器人尚未到达的目标处放置障碍。"""
        robot = self.robots[self.selected_robot]
        task = self.task_for(robot)
        if task is None or self.temp:
            self.emit("obstacle_rejected", reason="select_active_robot_and_clear_existing_obstacle")
            return False
        if any(math.dist(task.goal, r.xy) < 1.3 for r in self.robots) or any(
                math.dist(task.goal, a.xy) < 1.3 for a in self.ward.actors):
            self.emit("obstacle_rejected", reason="target_too_close_to_robot_or_person")
            return False
        box = Box(*task.goal, .7, .7, .8, (.8, .15, .75, 1), "test target blocker")
        box.body = box_body([*task.goal, .4], [.7, .7, .8], box.color)
        self.temp.append(box)
        self.blocked_target = task.task_id
        self.emit("obstacle_added", robot, task)
        self.replan_all()
        return True

    def clear_obstacles(self):
        for box in self.temp:
            p.removeBody(box.body)
        self.temp.clear()
        self.blocked_target = None
        self.emit("obstacles_cleared")
        self.schedule_timer = 1.0
        self.replan_all()

    def replan_all(self):
        for robot in self.robots:
            task = self.task_for(robot)
            goal = task.goal if task else robot.home
            if math.dist(robot.xy, goal) >= .025:
                self.replan_robot(robot, goal)

    def report(self):
        return dict(
            scenario=self.ward.kind, completed=self.done,
            task_count=len(self.tasks), completed_tasks=sum(t.status == "completed" for t in self.tasks),
            simulated_seconds=round(self.elapsed, 3),
            parallel_task_seconds=round(self.parallel_seconds, 3),
            simultaneous_motion_seconds=round(self.simultaneous_motion_seconds, 3),
            min_robot_center_distance_m=(round(self.min_robot_distance, 4)
                                         if math.isfinite(self.min_robot_distance) else None),
            robot_contact_threshold_m=2 * RADIUS,
            robot_overlap_frames=self.safety_violations,
            tasks=[asdict(t) for t in self.tasks],
            robots=[dict(robot_id=r.robot_id, status=r.status, position=r.xy,
                         task_id=r.task_id, completed_tasks=r.completed_tasks,
                         distance_m=round(r.distance, 3), wait_seconds=round(r.wait_seconds, 3),
                         replans=r.replans) for r in self.robots],
            events=self.events)

    def save_report(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.report(), ensure_ascii=False, indent=2), encoding="utf-8")

    def close(self):
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None
        if p.isConnected(self.client):
            p.disconnect(self.client)


def main():
    parser = argparse.ArgumentParser(description="Two robots, shared FIFO tasks and A* navigation")
    parser.add_argument("--scenario", choices=("corridor", "ring"), default="corridor")
    parser.add_argument("--beds", nargs="+", help="Ordered bed IDs; defaults to four patrol tasks")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=240, help="Headless simulation time limit")
    parser.add_argument("--no-people", action="store_true")
    parser.add_argument("--report", default="demo_outputs/dual_robot_report.json")
    parser.add_argument("--events", default="demo_outputs/dual_robot_events.jsonl")
    parser.add_argument("--screenshot", help="Save initial scene PNG (without GUI debug text)")
    parser.add_argument("--block-test", action="store_true",
                        help="Block R1 target at 1s, clear at 5s; deterministic recovery demo")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    try:
        sim = DualRobotSimulation(args.scenario, not args.headless, args.beds,
                                  not args.no_people, args.events)
    except ValueError as error:
        parser.error(str(error))
    exit_code = 0
    blocked = cleared = False
    try:
        if args.screenshot:
            sim.screenshot(args.screenshot)
        print("1/2: select robot | B: block its target | C: clear | R: replan | SPACE: pause | V: camera | L: save | ESC: exit")
        print("A* baseline; no PPO, priority, MQTT or Web integration.")
        while p.isConnected(sim.client):
            if args.headless and (sim.done or sim.elapsed >= args.seconds):
                exit_code = 0 if sim.done else 1
                break
            if args.block_test and not blocked and sim.elapsed >= 1:
                blocked = True
                if not sim.block_selected_target():
                    raise RuntimeError("Block-test obstacle could not be inserted")
            if args.block_test and not cleared and sim.elapsed >= 5:
                cleared = True
                sim.clear_obstacles()
            if not args.headless:
                keys = p.getKeyboardEvents()
                def pressed(key):
                    return keys.get(key, 0) & p.KEY_WAS_TRIGGERED
                if pressed(27):
                    break
                if pressed(ord("1")):
                    sim.selected_robot = 0
                if pressed(ord("2")):
                    sim.selected_robot = 1
                for key, action in (("b", sim.block_selected_target), ("c", sim.clear_obstacles),
                                    ("r", sim.replan_all), ("v", sim.camera),
                                    ("l", lambda: sim.save_report(args.report))):
                    if pressed(ord(key)) or pressed(ord(key.upper())):
                        action()
                if pressed(32):
                    sim.paused = not sim.paused
                    sim.emit("paused" if sim.paused else "resumed")
                if sim.paused or sim.done:
                    sim.refresh_ui()
            sim.tick()
            if not args.headless:
                time.sleep(DT)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        try:
            sim.emit("simulation_stopped", completed=sim.done)
            sim.save_report(args.report)
            print(f"Report: {args.report} | completed={sim.done} | {sum(t.status == 'completed' for t in sim.tasks)}/{len(sim.tasks)}")
        finally:
            sim.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
