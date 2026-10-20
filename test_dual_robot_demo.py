"""双机器人验收：真实仿真、任务唯一性、连续移动安全、暂停及受阻恢复。"""
from collections import Counter
import json
import math
from pathlib import Path
import tempfile
import unittest

import pybullet as p

from dual_robot_demo import Box, DT, RADIUS, DualRobotSimulation
from verify_dual_report import verify


class DualRobotTests(unittest.TestCase):
    def make_sim(self, **kwargs):
        sim = DualRobotSimulation(gui=False, verbose=False, **kwargs)
        self.addCleanup(sim.close)
        return sim

    def assert_safe(self, sim, physics=False):
        r1, r2 = sim.robots
        self.assertGreater(math.dist(r1.xy, r2.xy), 2 * RADIUS)
        for robot in sim.robots:
            self.assertTrue(sim.free(robot.xy, False), (robot.robot_id, robot.xy))
            for actor in sim.ward.actors:
                self.assertGreater(math.dist(robot.xy, actor.xy), RADIUS + actor.radius)
            if physics:
                for box in sim.ward.boxes + sim.temp:
                    self.assertFalse(p.getClosestPoints(robot.body, box.body, 0))
                for actor in sim.ward.actors:
                    self.assertFalse(p.getClosestPoints(robot.body, actor.body, 0))
        if physics:
            self.assertFalse(p.getClosestPoints(r1.body, r2.body, 0))

    def run_until_done(self, sim, seconds=240):
        for frame in range(int(seconds / DT)):
            sim.tick()
            self.assert_safe(sim, physics=frame % 30 == 0)
            active = [r.task_id for r in sim.robots if r.task_id is not None]
            self.assertEqual(len(active), len(set(active)))
            if sim.done:
                return
        self.fail(f"Timeout: {sim.report()['robots']}")

    def assert_task_history(self, sim):
        for name in ("task_created", "task_assigned", "task_arrived", "task_completed"):
            counts = Counter(e["task_id"] for e in sim.events if e["event"] == name)
            self.assertEqual(counts, Counter(t.task_id for t in sim.tasks), name)
        for task in sim.tasks:
            self.assertEqual(task.status, "completed")
            self.assertLessEqual(task.created_at, task.assigned_at)
            self.assertGreater(task.completed_at, task.assigned_at)
            for event in sim.events:
                if event.get("task_id") == task.task_id and "robot_id" in event:
                    self.assertEqual(event["robot_id"], task.robot_id)

    def test_default_patrol_both_layouts(self):
        for kind in ("corridor", "ring"):
            with self.subTest(kind=kind):
                sim = self.make_sim(kind=kind)
                try:
                    sim.tick()
                    self.assertEqual(sum(t.status == "assigned" for t in sim.tasks), 2)
                    self.assertEqual(sum(t.status == "pending" for t in sim.tasks), 2)
                    self.run_until_done(sim)
                    self.assert_task_history(sim)
                    self.assertTrue(all(r.completed_tasks > 0 for r in sim.robots))
                    self.assertGreater(sim.simultaneous_motion_seconds, 1)
                    self.assertEqual(sim.safety_violations, 0)
                    self.assertEqual(verify(sim.report()), [])
                finally:
                    sim.close()

    def test_blocked_goal_wait_clear_and_resume(self):
        sim = self.make_sim(people=False)
        sim.tick()
        self.assertTrue(sim.block_selected_target())
        blocked_task = sim.task_for(sim.robots[0])
        before = sim.robots[0].xy
        for _ in range(180):
            sim.tick()
            self.assert_safe(sim)
        self.assertEqual(sim.robots[0].xy, before)
        self.assertEqual(blocked_task.status, "assigned")
        self.assertGreater(sim.robots[0].wait_seconds, 0)
        sim.clear_obstacles()
        self.run_until_done(sim)
        self.assert_task_history(sim)
        names = [e["event"] for e in sim.events]
        self.assertIn("wait_started", names)
        self.assertIn("wait_ended", names)

    def test_head_on_motion_cannot_cross(self):
        sim = self.make_sim(people=False)
        first, second = sim.robots
        # 人工构造对向冲突，禁用重规划以单独验证移动安全闸。
        for robot, start, goal in ((first, (-6, 0), (-4, 0)),
                                   (second, (-4, 0), (-6, 0))):
            robot.xy, robot.home, robot.path = start, goal, [goal]
            robot.plan_timer = -10000
            p.resetBasePositionAndOrientation(robot.body, [*start, .245], [0, 0, 0, 1])
        for _ in range(180):
            for robot in sim.robots:
                sim.advance_robot(robot)
            self.assert_safe(sim, physics=True)
            self.assertLess(first.xy[0], second.xy[0])
        self.assertTrue(first.waiting and second.waiting)

    def test_pause_freezes_world_and_events(self):
        sim = self.make_sim()
        sim.tick()
        before = (sim.elapsed, [r.xy for r in sim.robots],
                  [a.xy for a in sim.ward.actors], len(sim.events))
        sim.paused = True
        for _ in range(100):
            sim.tick()
        self.assertEqual(before, (sim.elapsed, [r.xy for r in sim.robots],
                                 [a.xy for a in sim.ward.actors], len(sim.events)))

    def test_unreachable_pending_task_is_not_assigned(self):
        sim = self.make_sim(people=False)
        goal = sim.tasks[0].goal
        blocker = Box(*goal, .7, .7, .8, (1, 0, 0, 1))
        sim.temp.append(blocker)
        sim.dispatch()
        self.assertEqual(sim.tasks[0].status, "pending")
        self.assertIsNone(sim.tasks[0].robot_id)
        self.assertEqual(sum(t.status == "assigned" for t in sim.tasks), 2)
        sim.temp.remove(blocker)

    def test_report_verifier_rejects_incomplete_run(self):
        sim = self.make_sim(people=False)
        sim.tick()
        errors = verify(sim.report())
        self.assertIn("simulation not completed", errors)
        self.assertIn("missing or duplicate task_completed", errors)

    def test_dynamic_submission_and_same_bed_serialization(self):
        sim = self.make_sim(people=False, beds=["N1-1", "N1-1", "S3-2"])
        sim.tick()
        self.assertEqual([t.status for t in sim.tasks], ["assigned", "pending", "assigned"])
        extra = sim.add_task("N3-2")
        self.run_until_done(sim)
        self.assert_task_history(sim)
        self.assertEqual(extra.status, "completed")
        self.assertGreaterEqual(sim.tasks[1].assigned_at, sim.tasks[0].completed_at)
        sim.add_task("S1-1")
        self.assertFalse(sim.done)
        self.run_until_done(sim)
        self.assert_task_history(sim)
        count = len(sim.tasks)
        with self.assertRaises(ValueError):
            sim.add_task("INVALID")
        self.assertEqual(len(sim.tasks), count)

    def test_precise_corner_check_and_report_log(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "events.jsonl"
            report = Path(folder) / "report.json"
            sim = self.make_sim(people=False, event_log=log)
            # 此线段仅擦过膨胀墙角，粗粒度离散检查可能漏检。
            self.assertFalse(sim.segment((6.50, -2.50), (6.75, -2.75), False))
            sim.tick()
            sim.save_report(report)
            exported = json.loads(report.read_text(encoding="utf-8"))
            events = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(events, exported["events"])
            self.assertEqual(exported["completed_tasks"], 0)
            self.assertEqual(exported["task_count"], 4)
            sim.close()


if __name__ == "__main__":
    unittest.main()
