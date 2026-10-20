# 第二阶段 PPO 最小验证：验收指南

本阶段验收两件事：①真实 PPO 模型能训练、保存、重新加载，并决定机器人的局部动作；②有可复现的成功率、碰撞率、耗时对照记录。此阶段不要求已经完成双机器人 PPO 集成，也不预设 PPO 必须优于规则基线。

## 1. 检查文件和环境

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
.\venv\Scripts\python.exe -m pip check
Get-ChildItem ppo_artifacts
```

应看到：

- `ppo_patrol.zip`：训练完成的 PPO 模型。
- `ppo_static.zip`：静态预热模型。
- `training_config.json`：`completed` 为 true，记录实际步数和版本。
- `static_monitor.csv`、`dynamic_monitor.csv`、`progress.csv`：真实训练日志。
- `evaluation.json`、`evaluation.csv`、`evaluation.md`：批量评估记录。

依赖检查应输出 `No broken requirements found`。已有模型直接运行即可，不要为了验收先覆盖它。

## 2. 人工演示：模型加载后控制机器人

```powershell
.\venv\Scripts\python.exe ppo_patrol_demo.py --seed 1000
```

| 步骤 | 操作 | 预期结果 / 成效 |
| --- | --- | --- |
| 1 | 启动 | 终端打印 Controller=ppo；模型缺失时应明确报错，不能偷偷切换 A* 控制 |
| 2 | 观察场景 | 红色机器人、橙色人员、绿色目标及 A* 路线可见 |
| 3 | 观察移动 | 机器人位置由逐步动作更新；上方显示 LOCAL ACTION 编号及仿真时间 |
| 4 | 按空格暂停，再恢复 | 机器人、人员与仿真时间一起停止/继续 |
| 5 | 等回合结束 | 终端打印 success、static_collision、person_collision 或 timeout；失败也有记录 |
| 6 | 按 ESC | 退出并写入 `demo_outputs/ppo_demo_report.json`，包含动作轨迹 |

种子决定起终点和人员运动。同一个种子重复运行确定性策略应得到相同动作与结果。一个成功视频不能替代批量评估；一个失败回合也不应被从统计中删除。

如要先检查静态路点跟随：

```powershell
.\venv\Scripts\python.exe ppo_patrol_demo.py --mode static --seed 1000
```

## 3. 自动演示及轨迹核对

```powershell
.\venv\Scripts\python.exe ppo_patrol_demo.py --headless --seed 1000 --report demo_outputs/ppo_demo_report.json --screenshot demo_outputs/ppo_initial.png
$ppoDemo = Get-Content demo_outputs/ppo_demo_report.json -Raw | ConvertFrom-Json
$ppoDemo | Select-Object controller, mode, seed, outcome, success, collision, simulated_seconds, distance_m
$ppoDemo.trajectory | Select-Object -First 10 | Format-Table step, action, x, y, reward
```

通过标准：

- `controller=ppo`，存在多步轨迹，位置随动作改变。
- 动作由加载模型的 `predict()` 返回，并直接送到 `env.step(action)`。
- 成功回合 `success=true`；碰撞回合 `collision=true` 且不得算成功。
- 等待动作 0 不主动移动机器人；非等待动作方向与定义一致。
- 回合成功返回 0；碰撞/超时/中断返回 1。可以用 `$LASTEXITCODE` 查看刚执行的程序退出码。

动作是否真实控制、反向动作是否真的反向、碰撞是否被如实记录，都由测试覆盖，不仅靠画面观察。

## 4. 对照评估

```powershell
.\venv\Scripts\python.exe evaluate_ppo_patrol.py --episodes 20 --seed 2000 --output demo_outputs/ppo_recheck.json
```

该命令不训练、不修改模型，只执行评估，产生 JSON、CSV、Markdown。需要比较更多场景时可以改为 `--episodes 50 --seed 3000`，但应把它记录为另一批实验。

查看摘要：

```powershell
$ppoEvaluation = Get-Content demo_outputs/ppo_recheck.json -Raw | ConvertFrom-Json
$ppoEvaluation.summary | Format-Table mode, controller, episodes, successes, success_rate, collision_rate, timeout_rate, mean_success_seconds
```

验收要点：

| 检查 | 通过标准 |
| --- | --- |
| 对照组 | static/dynamic × ppo/astar/random，总计 6 组 |
| 场景一致 | 同一模式下，各控制器使用相同种子集合 |
| 数据完整 | 默认每组 20 回合，共 120 条原始记录 |
| 结果互斥 | 每个回合只有成功、碰撞、超时之一 |
| 比例一致 | 成功率＋碰撞率＋超时率=1 |
| 耗时解释 | 区分全部回合平均耗时与仅成功回合耗时 |
| 模型可追溯 | JSON 中有 model_sha256，能对应交付模型 |
| 控制记录 | 原始回合包含 action_counts，演示报告包含逐步轨迹 |

本项目的 A* 对照包含一步安全动作选择，并使用人员下一步位置的理想信息。PPO 也获得已知轨迹下的 0.6 秒风险特征，基线检查窗口为 0.2 秒；它们不是相同预测窗口的纯算法消融。对照名称必须保留这个含义，不能简称为“完全不避障的 A*”。

碰撞率是解析几何判定的碰撞动作尝试率。程序在碰撞后停止在前一个安全位置并结束，所以画面上可能没有明显重叠；这不意味着碰撞率应算 0。

## 5. 测试环境、模型和旧功能

```powershell
.\venv\Scripts\python.exe -m unittest test_ppo_patrol -v
```

应为 7 项测试、`OK`。完整模型测试要求模型和评估文件存在；缺文件应失败，不能静默跳过。

| 测试 | 验证内容 |
| --- | --- |
| Gymnasium/SB3 contract | 观测、奖励、结束标志符合环境接口 |
| Seed/action reproducibility | 相同种子和动作序列产生相同观测 |
| Wait/opposite action | 等待不动，反向动作实际反向，没有 A* 自动代走 |
| Static collision | 碰撞动作直接记失败，没有规则替换后继续 |
| Relative motion/timeout | 人员与机器人扫掠碰撞检查，以及 180 步超时 |
| A* route geometry | 全局路线符合静态几何安全条件 |
| Saved model/evaluation | 模型可加载并实际移动；评估结构和种子一致 |

旧版本回归：

```powershell
.\venv\Scripts\python.exe -m unittest test_dual_robot_demo test_complex_ward_demo -v
```

应为原有 10 项测试、`OK`。新增 NumPy 会改变 PyBullet 相机像素的返回类型，但旧截图代码已经兼容数组；回归检查确认旧导航仍可运行。

## 6. 重新训练验收（可选，耗时更长）

```powershell
.\venv\Scripts\python.exe train_ppo_patrol.py --steps 40000 --seed 7 --output ppo_artifacts_retrain
.\venv\Scripts\python.exe evaluate_ppo_patrol.py --model ppo_artifacts_retrain/ppo_patrol.zip --output ppo_artifacts_retrain/evaluation.json
.\venv\Scripts\python.exe ppo_patrol_demo.py --headless --model ppo_artifacts_retrain/ppo_patrol.zip --report demo_outputs/ppo_retrain_demo.json
```

观察训练日志更新，确认模型文件可加载。短训练如 `--steps 2048` 仅用于安装/流程冒烟检查，不能期待同样性能。训练脚本固定 CPU，不需要 CUDA。

## 7. 结果、边界和进入第三步的判断

实际成功率和失败记录以 [ppo_artifacts/evaluation.md](ppo_artifacts/evaluation.md) 及原始 JSON/CSV 为准。训练配置中的 `actual_steps` 是实际采样步数，`training_wall_seconds` 是本次实际训练用时，不能把它和单回合仿真秒混为一谈。

如果模型能够加载、控制移动，且有完整对照记录，第二阶段的最小工程验证成立。若动态避障仍有失败，第三阶段集成时应继续调优，并保留安全协调机制；不能将当前单机器人策略直接宣布为双机器人安全策略。

当前局限：单机器人、简化家具、单个规则运动人员、理想状态观测、运动学位置控制、小样本评估。没有临床监测、真实传感器、多智能体训练或复杂病区部署。

自动化测试、无窗口演示和离屏图片检查可以由开发过程完成；实际 GUI 按键和现场演示请按本指南手动复核。不要将无窗口结果描述为已完成所有 GUI 交互验收。

## 8. 本次实际交付与复核记录（2026-10-20）

- v2 模型训练已完成：40,448 步，包含 5,120 步静态预热，训练配置与两个阶段的原始日志均已保留。
- 重新加载模型后，动态演示种子 1000 成功到达：41 步、8.2 仿真秒、6.56 米，轨迹在 `demo_outputs/ppo_demo_report.json`。
- 最终批量评估使用 2000—2019，共 120 回合；复跑的全部回合结果、动作次数与原报告一致（实际电脑用时除外）。
- PPO 静态成功率 100%（20/20）；动态成功率 95%（19/20）。动态失败种子为 2005，原因是 static_collision，已保留，没有删除失败样本。
- 动态 PPO 的动作统计为 `[4, 713, 1, 30, 0, 0, 35, 0, 162]`，分别对应动作 0—8，说明策略会等待和偏转，而非始终直行。
- 依赖检查通过；结果图保存在 `ppo_artifacts/results.png`，由真实训练和评估数据生成。
- 新旧合并测试结果：`Ran 17 tests in 96.807s`，`OK`。完整记录在 `demo_outputs/ppo_acceptance_tests.txt`，包含 7 项 PPO 测试和 10 项旧版回归测试。

推荐展示具有偏转动作的成功回合：

```powershell
.\venv\Scripts\python.exe ppo_patrol_demo.py --seed 2000 --report demo_outputs/ppo_steering_demo.json
```

种子 2002 的成功回合还包含等待动作。要如实展示当前缺陷，可运行种子 2005，预期碰撞失败而非成功退出：

```powershell
.\venv\Scripts\python.exe ppo_patrol_demo.py --seed 2005 --report demo_outputs/ppo_failure_demo.json
```

本阶段交付的是可训练、可加载、可复核的最小 PPO 原型。动态成功率与当前 A* 规则基线相同、成功回合平均耗时略长，不宣称优于传统方法。
