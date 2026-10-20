"""第三阶段接口、策略控制权、安全闸、生命周期与实模型验收。"""
import math
from pathlib import Path
import unittest
import numpy as np
from stable_baselines3.common.env_checker import check_env
from dual_ppo_env import DualPatrolPPOEnv
from dual_ppo_demo import load_policy, run_episode
from complex_ward_demo import RADIUS
from verify_dual_report import verify


class DualPPOTests(unittest.TestCase):
    def make(self, **kwargs):
        env = DualPatrolPPOEnv(**kwargs)
        self.addCleanup(env.close)
        return env

    def test_gym_contract_seed_and_reset(self):
        env = self.make()
        check_env(env, warn=True)
        first, _ = env.reset(seed=10)
        env.step([1, 8])
        second, _ = env.reset(seed=10)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(first.shape, (52,))
        with self.assertRaises(ValueError):
            env.step([1, 9])

    def test_wait_action_really_stops_both_and_pause_freezes(self):
        env = self.make(people=False)
        env.reset(seed=1)
        before = [r.xy for r in env.sim.robots]
        for _ in range(5):
            env.step([0,0])
        self.assertEqual(before, [r.xy for r in env.sim.robots])
        self.assertGreater(sum(env.sim.policy_wait_frames), 0)
        env.step([1,1])
        self.assertNotEqual(before, [r.xy for r in env.sim.robots])
        env.sim.paused = True
        elapsed = env.sim.elapsed
        env.sim.tick()
        self.assertEqual(elapsed, env.sim.elapsed)

    def test_random_actions_cannot_cross_or_enter_obstacles(self):
        env = self.make(max_seconds=12)
        env.reset(seed=42)
        rng = np.random.default_rng(19)
        for _ in range(60):
            _, _, terminated, truncated, _ = env.step(rng.integers(0,9,size=2))
            sim = env.sim
            self.assertGreater(math.dist(*[r.xy for r in sim.robots]), 2*RADIUS)
            for r in sim.robots:
                self.assertTrue(sim.free(r.xy, False))
                for actor in sim.ward.actors:
                    self.assertGreater(math.dist(r.xy, actor.xy), RADIUS+actor.radius)
            active = [r.task_id for r in sim.robots if r.task_id]
            self.assertEqual(len(active), len(set(active)))
            if terminated or truncated:
                break
        self.assertTrue(truncated)
        with self.assertRaises(RuntimeError):
            env.step([1,1])

    def test_safety_rejects_head_on_without_action_replacement(self):
        env = self.make(people=False)
        env.reset(seed=0)
        a,b = env.sim.robots
        a.xy, b.xy = (-5.,0.), (-4.2,0.)
        a.path, b.path = [b.xy], [a.xy]
        a.plan_timer = b.plan_timer = -1000
        env.sim.actions = [1,1]
        for _ in range(120):
            for robot in env.sim.robots:
                env.sim.advance_robot(robot)
            self.assertLess(a.xy[0], b.xy[0])
            self.assertGreater(math.dist(a.xy,b.xy),2*RADIUS)
        self.assertGreater(sum(env.sim.blocked_frames),0)

    def test_blocked_target_waits_and_recovers_after_clear(self):
        env = self.make(people=False)
        env.reset(seed=0)
        sim = env.sim
        self.assertTrue(sim.block_selected_target())
        task = sim.task_for(sim.robots[0])
        before = sim.robots[0].xy
        for _ in range(10):
            env.step([1,1])
        self.assertEqual(before, sim.robots[0].xy)
        self.assertEqual(task.status, "assigned")
        sim.clear_obstacles()
        for _ in range(900):
            _, _, terminated, truncated, _ = env.step([1,1])
            if terminated or truncated:
                break
        self.assertTrue(sim.done)
        self.assertEqual(verify(sim.report()), [])

    def test_reverse_action_changes_direction(self):
        env = self.make(people=False)
        env.reset(seed=0)
        robot = env.sim.robots[0]
        np.testing.assert_allclose(env.sim.delta(robot,5), -env.sim.delta(robot,1), atol=1e-12)

    def test_delivered_model_completes_and_old_model_rejected(self):
        model, sha = load_policy("dual_ppo_artifacts/ppo_dual.zip")
        self.assertEqual(len(sha),64)
        report = run_episode(model=model, people=False, seed=3000)
        self.assertTrue(report["completed"], report["robots"])
        self.assertEqual(verify(report), [])
        self.assertTrue(all(r["completed_tasks"] > 0 for r in report["robots"]))
        actions = [row["actions"] for row in report["trajectory"]]
        self.assertTrue(all(len(a) == 2 for a in actions))
        if Path("ppo_artifacts/ppo_patrol.zip").exists():
            with self.assertRaises(ValueError):
                load_policy("ppo_artifacts/ppo_patrol.zip")


if __name__ == "__main__":
    unittest.main()
