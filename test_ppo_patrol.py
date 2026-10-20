"""第二阶段验收：环境协议、动作真实性、碰撞、复现、模型与评估记录。"""
import json
import hashlib
from pathlib import Path
import unittest

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import torch

from ppo_patrol_env import PatrolPPOEnv


class PPOEnvironmentTests(unittest.TestCase):
    def make_env(self, mode="dynamic"):
        env=PatrolPPOEnv(mode)
        self.addCleanup(env.close)
        return env

    def test_gymnasium_and_sb3_contract(self):
        env=self.make_env()
        check_env(env,warn=True)

    def test_seed_and_action_reproducibility(self):
        env=self.make_env()
        def rollout():
            obs,_=env.reset(seed=23)
            rows=[obs]
            for action in [1,0,2,1,8,1]:
                obs,_,done,timeout,_=env.step(action)
                rows.append(obs)
                if done or timeout:break
            return np.array(rows)
        np.testing.assert_array_equal(rollout(),rollout())

    def test_wait_and_opposite_action_really_control_motion(self):
        env=self.make_env("static")
        env.reset(seed=1000)
        start=env.xy.copy()
        env.step(0)
        np.testing.assert_array_equal(start,env.xy)
        forward=env.action_delta(1)
        reverse=env.action_delta(5)
        np.testing.assert_allclose(forward,-reverse,atol=1e-10)
        env.cached_risks[5]=-1  # 风险标志只能影响奖励，不能屏蔽/替换动作。
        env.step(5)
        np.testing.assert_allclose(env.xy,start+reverse)

    def test_static_collision_is_failure_not_hidden_safety_override(self):
        env=self.make_env("static")
        env.reset(seed=1)
        env.xy=np.array([-.90,0.])
        # 向床中心的动作必须失败，不允许替换成其他安全动作。
        env.waypoint=lambda: np.array([1.,0.])
        _,reward,terminated,truncated,info=env.step(1)
        self.assertTrue(terminated)
        self.assertFalse(truncated)
        self.assertTrue(info["collision"])
        self.assertEqual(info["outcome"],"static_collision")
        self.assertLess(reward,0)
        self.assertEqual(info["controller_action"],1)
        with self.assertRaises(RuntimeError):env.step(0)

    def test_relative_motion_collision_and_timeout(self):
        env=self.make_env()
        env.reset(seed=1)
        env.xy=np.array([-3.,0.])
        env.person=np.array([-3.,-.8])
        self.assertEqual(env.collision_for(env.xy,np.array([-3.,.8])),"person_collision")
        env.mode="static"
        env.steps=env.MAX_STEPS-1
        _,_,terminated,truncated,info=env.step(0)
        self.assertFalse(terminated)
        self.assertTrue(truncated)
        self.assertEqual(info["outcome"],"timeout")

    def test_astar_routes_clear_static_geometry(self):
        env=self.make_env("static")
        for seed in range(12):
            env.reset(seed=seed)
            self.assertTrue(all(env.static_segment(a,b,.08)
                                for a,b in zip(env.route,env.route[1:])))

    def test_model_load_predict_and_saved_evaluation(self):
        path=Path("ppo_artifacts/ppo_patrol.zip")
        self.assertTrue(path.exists(),"Run train_ppo_patrol.py before full acceptance")
        torch.set_num_threads(1)
        model=PPO.load(path,device="cpu")
        env=self.make_env()
        obs,_=env.reset(seed=1000)
        initial=env.xy.copy()
        for _ in range(30):
            action=int(model.predict(obs,deterministic=True)[0])
            obs,_,done,timeout,info=env.step(action)
            self.assertEqual(info["controller_action"],action)
            if done or timeout:break
        self.assertGreater(np.linalg.norm(env.xy-initial),.1)
        result=json.loads(Path("ppo_artifacts/evaluation.json").read_text(encoding="utf-8"))
        self.assertEqual(result["env_version"],env.VERSION)
        self.assertEqual(result["model_sha256"],hashlib.sha256(path.read_bytes()).hexdigest())
        self.assertEqual(len(result["summary"]),6)
        for row in result["summary"]:
            self.assertAlmostEqual(row["success_rate"]+row["collision_rate"]+row["timeout_rate"],1)
            episodes=[r for r in result["episodes"] if r["mode"]==row["mode"] and r["controller"]==row["controller"]]
            self.assertEqual(len(episodes),row["episodes"])
            self.assertEqual(sum(r["success"] for r in episodes),row["successes"])
            self.assertAlmostEqual(sum(r["collision"] for r in episodes)/len(episodes),row["collision_rate"])
            for episode in episodes:
                self.assertEqual(sum(episode["action_counts"]),episode["steps"])
        for mode in ("static","dynamic"):
            seeds=[{r["seed"] for r in result["episodes"] if r["mode"]==mode and r["controller"]==c}
                   for c in ("ppo","astar","random")]
            self.assertEqual(seeds[0],seeds[1])
            self.assertEqual(seeds[1],seeds[2])


if __name__=="__main__":
    unittest.main()
