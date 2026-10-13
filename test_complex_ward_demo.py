"""只测试新增 demo：连通性、受阻恢复、动态巡查及几何安全距离。"""
import math
import unittest

import pybullet as p
from complex_ward_demo import Box, DT, RADIUS, Simulation, Ward


class WardTests(unittest.TestCase):
    def test_static_connectivity(self):
        for kind in ('corridor', 'ring'):
            with self.subTest(kind=kind):
                sim = Simulation(Ward(kind), False)
                try:
                    start = sim.xy
                    for name, goal in sim.ward.stops:
                        # 按实际巡检顺序验证床位之间连通，暂时忽略移动人员。
                        path = sim.plan(start, goal, dynamic=False)
                        self.assertTrue(path, name)
                        self.assertTrue(all(sim.segment(a, b, False) for a, b in zip(path, path[1:])), name)
                        start = goal
                    for actor in sim.ward.actors:
                        a, b = actor.route
                        for i in range(101):
                            xy = tuple(a[k] + (b[k] - a[k]) * i / 100 for k in (0, 1))
                            self.assertTrue(sim.free(xy, False, actor.radius + .05))
                finally:
                    p.disconnect(sim.client)

    def test_dynamic_patrol_and_temporary_obstacles(self):
        for kind in ('corridor', 'ring'):
            with self.subTest(kind=kind):
                sim = Simulation(Ward(kind), False)
                try:
                    sim.replan()
                    self.assertTrue(sim.add_obstacle())
                    # 完全覆盖第一个巡检点时，必须报告无路，不能穿障。
                    goal = sim.ward.stops[0][1]
                    blocker = Box(*goal, .6, .6, .8, (1, 0, 0, 1))
                    sim.temp.append(blocker)
                    self.assertFalse(sim.plan(sim.xy, goal))
                    sim.replan()
                    before = sim.xy
                    for _ in range(80):
                        sim.tick()
                    self.assertEqual(sim.xy, before)
                    self.assertFalse(sim.path)
                    # 移除堵住目标的障碍，检查机器人能否自动恢复并完成巡检。
                    sim.temp.remove(blocker)
                    for frame in range(int(240 / DT)):
                        if frame == 600:
                            sim.clear_obstacles()
                        sim.tick()
                        self.assertTrue(sim.free(sim.xy, dynamic=False), (kind, sim.xy))
                        for actor in sim.ward.actors:
                            self.assertGreater(math.dist(sim.xy, actor.xy), RADIUS + actor.radius)
                        # 除了坐标检查，还定期用仿真引擎确认机器人没有碰到家具。
                        if frame % 60 == 0:
                            for b in sim.ward.boxes + sim.temp:
                                self.assertFalse(p.getClosestPoints(sim.robot, b.body, 0), b.name)
                        if sim.done:
                            break
                    self.assertTrue(sim.done, sim.report())
                    self.assertEqual(len(sim.events), len(sim.ward.stops))
                    self.assertEqual(len({e['bed'] for e in sim.events}), len(sim.events))
                finally:
                    p.disconnect(sim.client)


if __name__ == '__main__':
    unittest.main()
