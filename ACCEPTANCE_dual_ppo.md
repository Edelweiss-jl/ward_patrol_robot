# 第三阶段验收

在项目根目录使用现有 `venv\Scripts\python.exe`，详细说明见 [README_dual_ppo.md](README_dual_ppo.md)。

1. 运行 `python -m unittest test_dual_ppo`（将python换成venv解释器）。应检查52维Gym契约、固定种子重置、两台等待动作真实停车、主动前进、暂停、随机动作安全、对向冲突被拦截、超时生命周期、实模型完成任务以及拒绝第二阶段模型。
2. 运行 `.\venv\Scripts\python.exe dual_ppo_demo.py --headless --no-people`。应成功退出0，两台各有完成任务，报告 `completed=true`、四任务均完成、`robot_overlap_frames=0`；轨迹每条有两个动作。
3. 运行 `.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_ppo_report.json`。应通过任务唯一分配、唯一完成与事件顺序校验。
4. 运行 `.\venv\Scripts\python.exe dual_ppo_demo.py`。查看红蓝两台机器人执行任务；按空格应同时暂停人员与机器人，再按继续，ESC退出应保存报告。提前退出通常返回1。
5. 运行 `.\venv\Scripts\python.exe dual_ppo_demo.py --headless --seconds 0.2 --report demo_outputs/dual_ppo_timeout.json`。应返回1并记录timeout，不能误报全部完成。
6. 运行 `.\venv\Scripts\python.exe dual_ppo_demo.py --model ppo_artifacts/ppo_patrol.zip --headless`。应报观测/动作空间不匹配，返回2，不会降级为A*。
7. 运行 `.\venv\Scripts\python.exe evaluate_dual_ppo.py --episodes 5`。检查JSON/CSV/Markdown的组数、种子、成功数、超时、模型哈希；不能提前假定所有组成功。与交付评估一致地解释安全层作用。
8. 运行 `.\venv\Scripts\python.exe train_dual_ppo.py --steps 512 --output demo_outputs/dual_ppo_smoke`。应有真实模型和训练CSV，配置completed=true。再次相同输出命令应拒绝覆盖。512步只用于流程验收，不是性能验收。
9. 回归运行 `.\venv\Scripts\python.exe -m unittest test_complex_ward_demo test_dual_robot_demo test_ppo_patrol`，确认前两阶段继续可用。

危险动作测试的正确结果是停车并增加拦截帧数，不能要求“完全没有拦截”。安全拦截不是物理碰撞次数，也不能作为策略已学会可靠避障的证据。
