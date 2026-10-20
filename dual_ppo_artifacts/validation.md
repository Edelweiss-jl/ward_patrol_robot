# 本次验证记录

日期：2026-10-20。环境：项目现有Python 3.10 venv；依赖版本见training_config.json。

| 检查 | 实际结果 |
| --- | --- |
| `python -m unittest test_dual_ppo` | 7项通过，包括真实交付模型完成、旧模型拒绝、受阻恢复和Gym检查 |
| `python -m unittest test_complex_ward_demo test_dual_robot_demo test_ppo_patrol` | 17项通过 |
| `python -m pip check` | No broken requirements found |
| `python dual_ppo_demo.py --headless` | 默认动态走廊4/4完成，75.55仿真秒 |
| `python verify_dual_report.py demo_outputs/dual_ppo_report.json` | PASS；同时移动45.4秒，最小机器人中心距离2.0米 |
| 0.2秒上限演示 | timeout，0/4任务，未误报成功 |
| 第二阶段模型传入第三阶段入口 | 以52维观测/联合动作空间错误拒绝 |
| `python evaluate_dual_ppo.py --episodes 5` | 40回合完整保存JSON、CSV、Markdown；包含超时记录 |
| `python plot_dual_ppo_results.py` | 生成results.png，已检查图面 |

以上命令均使用项目 `venv\Scripts\python.exe`。GUI交互未做人工点击验收；已通过无窗口仿真和截图检查。

PPO成功16/20，A*成功16/20；环形动态均为1/5。确定性PPO动作全部为前进，未验证学到主动协作避障。完整解读见上一级README_dual_ppo.md第8节，原始记录和模型哈希见evaluation.json。
