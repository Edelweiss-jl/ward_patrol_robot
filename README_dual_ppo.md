# 第三阶段：双机器人接入 PPO

本阶段把真正的 Stable-Baselines3 PPO 接入第一阶段的复杂病区和双机器人任务队列。一个联合策略读取两台机器人的观测，一次输出两台机器人的动作；FIFO 调度器继续分配任务，A* 继续提供全局路点，PPO 决定等待、前进或偏转。保留第一、第二阶段的源码和模型，新增第三阶段入口。

这是**集中式联合 PPO**：单个 Gymnasium 环境、52 维联合观测、`MultiDiscrete([9,9])` 联合动作、团队奖励。不是 MAPPO，也不是两台独立 PPO；没有接入 MQTT、数据库、体征、Web 或真实机器人。

## 1. 快速运行

PowerShell 在项目目录执行：

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
.\venv\Scripts\python.exe dual_ppo_demo.py
```

默认加载 `dual_ppo_artifacts/ppo_dual.zip`，复杂走廊病区、两台机器人、移动人员、四个巡查任务、种子 3000。红色 R1、蓝色 R2；任务信息与路径显示沿用第一阶段。启动后自动执行，不必重新训练。空格暂停/继续，ESC 提前退出；本入口在成功、超时或 ESC 后关闭窗口并保存报告。运行时只有一个 PyBullet 世界，不能同时在同一进程创建第二个环境。

PyCharm：解释器选 `venv\Scripts\python.exe`，运行文件选 `dual_ppo_demo.py`，工作目录设为项目根目录。

无窗口运行及截图：

```powershell
.\venv\Scripts\python.exe dual_ppo_demo.py --headless --screenshot demo_outputs/dual_ppo_initial.png
.\venv\Scripts\python.exe dual_ppo_demo.py --headless --no-people --report demo_outputs/dual_ppo_static.json
.\venv\Scripts\python.exe dual_ppo_demo.py --controller astar --headless --report demo_outputs/dual_astar.json
.\venv\Scripts\python.exe dual_ppo_demo.py --headless --scenario ring --report demo_outputs/dual_ppo_ring.json
```

默认报告 `demo_outputs/dual_ppo_report.json`，重复运行覆盖同名报告。退出码：全部任务完成为 0，超时或 ESC 中断为 1，模型/参数错误为 2。`--seconds` 是仿真时间上限，默认 180 秒，不是实际电脑用时。模型缺失会报错；不会自动切换控制器。第二阶段 28 维单机器人模型不能用于本入口。

自定义任务与时间上限：

```powershell
.\venv\Scripts\python.exe dual_ppo_demo.py --headless --beds N1-1 S3-2 N2-1 S2-2 --seconds 240 --seed 3010 --report demo_outputs/dual_ppo_custom.json
```

走廊床位：N1-1、N1-2、N2-1、N2-2、N3-1、N3-2、S1-1、S1-2、S2-1、S2-2、S3-1、S3-2。环形床位：ICU1—ICU4、WARD1—WARD4。重复床位任务会按原调度器串行执行，避免同一床旁同时有两个任务。

## 2. 控制链路与责任

```text
共享任务队列 -> FIFO + 静态 A* 路长选空闲机器人
                 |
           R1 / R2 各自目标
                 |
        A* 路径（包含动态障碍重规划）
                 |
       52维联合观测 -> 已训练 PPO -> [R1动作, R2动作]
                 |
       每帧连续几何安全闸 -> 提交移动或停车
                 |
       到床停留0.6秒 -> 完成任务 -> 领取后续任务
```

`PPODualSimulation` 继承原双机器人仿真，重写 `advance_robot()`；任务分配、人员礼让、任务唯一性、到床停留、返航和事件生命周期继续复用。仅在已到目标时调用父类处理停留或停车，父类不会在该分支执行导航。

模型每 0.2 仿真秒选择一次动作，底层仍按 1/60 秒更新，约 12 帧共用同一动作编号。每帧朝当前第一个未到达 A* 路点重新计算参考方向。R1、R2 顺序提交位置，R2 看到 R1 本帧的新位置，禁止相互穿过、直接交换位置。人员先更新，继续沿用同时避让两台机器人的规则。

## 3. 动作、观测与安全层

每台机器人有 9 个动作：0 等待；1 朝当前 A* 路点移动；2—8 将参考方向依次旋转 45、90、135、180、225、270、315 度。速度 0.95 米/秒；每帧步长不超过到当前路点的距离。动作 0 确实停止移动，不会被 A* 自动前进覆盖。无路时等待并按原规则重规划。

每台机器人 26 维观测，按 R1、R2 拼接为 52 维：

| 内容 | 维数 | 语义 |
| --- | --- | --- |
| 当前路点方向 | 2 | 单位方向；无路径时指向目标 |
| 最终目标相对偏移 | 2 | 除以20米 |
| 自身位置 | 2 | 除以病区半宽/半高 |
| 另一台机器人相对位置 | 2 | 除以20米 |
| 另一台上一控制周期平均速度 | 2 | 米/秒 |
| 自己、另一台是否持有任务 | 2 | 0或1 |
| 最近人员相对位置 | 2 | 除以20米；无人时为0 |
| 最近人员朝当前路点的名义速度 | 2 | 不等于礼让后的实际速度；无人时为0 |
| 是否存在人员 | 1 | 0或1 |
| 九种动作的当前几何安全特征 | 9 | +1安全，-1受阻 |

全部裁剪到 [-1,1]，float32。安全特征按当前人员/另一台机器人位置检查一个控制周期的候选线段，不预测未来轨迹，不是动作掩码。观测使用仿真真值，没有定位误差、传感噪声。只有最近人员的显式状态，但几何检查包含所有人员。

安全闸沿用连续线段与膨胀矩形/圆检查，使用机器人半径0.28米及0.16米余量。危险动作**只会停车并记录拦截**，不会替换成其他移动动作。重规划改变全局参考路线，不能替 PPO 选择局部动作。

这与第二阶段“危险动作终止回合”的定义不同：第三阶段测试的是 **PPO + A* + 安全层** 的系统。无碰撞不能单独归因于 PPO 学会避障。狭窄通道可能持续等待；没有通道预约、倒车协商或全局死锁消解，超时照实计失败。

## 4. 训练方法与复现

依赖复用 `requirements_ppo.txt`，本机 venv 已安装；换环境时执行：

```powershell
.\venv\Scripts\python.exe -m pip install -r requirements_ppo.txt --extra-index-url https://download.pytorch.org/whl/cpu
.\venv\Scripts\python.exe -m pip check
```

重新训练请使用新的输出目录：

```powershell
.\venv\Scripts\python.exe train_dual_ppo.py --steps 16384 --seed 17 --scenario corridor --output dual_ppo_artifacts_new
```

CPU、单 PyTorch 线程、两层64单元 MLP、rollout512、batch64、6个epoch、学习率0.0001、gamma0.99、熵系数0.005。实际步数按512取整；最低允许512步。输出目录非空默认拒绝覆盖，需要替换时显式 `--overwrite`。Ctrl+C 尽量保存 `ppo_interrupted.zip`，配置中标记未完成，不能当完整交付模型。

**显式初始化先验**：训练前将两台机器人动作1的初始 logit bias 设为3，其余为0，随后全部参数通过真正 PPO rollout 和梯度更新。没有行为克隆或专家轨迹。这个先验鼓励沿路点前进，因此不能仅凭确定性演示顺利完成就证明策略学会了双机器人协作；应结合动作分布和独立评估判断。

每回合保持同一四任务集合，随机排列队列；随机化人员在其往返路线上的初始位置及速度（原速度的0.85—1.15倍）。地图、机器人起点和四任务集合固定，不代表任意任务或地图泛化。训练默认只覆盖走廊，环形用于跨布局验证；需要环形专用策略可另用 `--scenario ring` 训练。

团队奖励按每个0.2秒控制周期计算：

```text
4 × 两台机器人累计路点距离减少量
- 0.03 时间惩罚
- 0.03 × 本周期安全拦截仿真帧数
+ 15 × 新完成任务数
+ 20（整批完成）
- 10（超时）
```

距离减少量在每帧针对当时的当前路点计算；返航也包含在内。奖励不是路径最优性的证明。回合只有全部任务完成或达到时间上限才结束；不会因为安全层拦截就宣称发生实际碰撞。

训练产物：`ppo_dual.zip`、`training_config.json`（版本、种子、配置、实际步数、完成标志、实际用时）、`monitor.csv`（每回合奖励/长度）、`progress.csv`（PPO优化日志）。交付模型的准确训练信息以配置文件为准。

## 5. 公平对照评估

```powershell
.\venv\Scripts\python.exe evaluate_dual_ppo.py --episodes 5 --seed 3000 --output dual_ppo_artifacts/evaluation.json
```

默认评估走廊/环形、有人员/无人员、PPO/原版 A*，每组5回合，共40回合。每组种子3000—3004，与训练种子17分开；两控制器使用完全相同的 reset 来生成任务顺序和人员初始条件。人员之后会礼让机器人，因此实际轨迹可随控制器行为变化。A* 基线绑定回原版 `advance_robot()`，不是简单地把 PPO 动作固定为1。

输出 JSON 原始逐回合指标与分组摘要、CSV、Markdown；记录模型 SHA256 和时间上限。PPO 用确定性推理，不在评估时学习。模型训练布局之外的结果需要单独解释。

| 指标 | 正确解释 |
| --- | --- |
| success_rate | 全部任务在时限内完成的回合比例 |
| mean_success_seconds | 仅成功回合平均完成时间；无成功为null |
| completed_tasks/task_count | 完成任务数；部分完成不算成功回合 |
| safety_rejected_frames | PPO动作被安全闸停车的底层帧数；一次动作最多贡献约12帧 |
| robot_overlap_frames | 仿真帧末两台机器人中心距离不大于两倍半径的帧数 |
| min_robot_center_distance_m | 帧末最小机器人中心距离 |
| distance_m | 两台累计运动距离，含返航 |

A* 基线的 `safety_rejected_frames` 是0，因为原控制器没有第三阶段的拦截计数器；不能把该字段直接用于 PPO/A* 安全优劣排名。A* 的等待可看原任务报告 `robots[].wait_seconds`；成功率与成功耗时可以比较。机器人重叠指标不涵盖所有墙体/人员碰撞，自动测试还会检查几何间距。

本次实测表见 [evaluation.md](dual_ppo_artifacts/evaluation.md)，原始记录见同目录JSON/CSV；不要把训练奖励或少量成功演示写成“PPO优于A*”。

汇报图见 [results.png](dual_ppo_artifacts/results.png)，由真实训练/评估记录生成，包含奖励、成功率、成功耗时和确定性动作分布。重新评估后执行 `.\venv\Scripts\python.exe plot_dual_ppo_results.py` 更新。

## 6. 报告与代码结构

报告保留第一阶段 `tasks`、`robots`、`events`、并行持有任务时间、同时移动时间和完成统计，增加环境版本、安全拦截、策略主动等待、控制器和种子。PPO 报告还有模型路径、SHA256。`trajectory` 每控制周期记录两台动作、位置、奖励和累计拦截帧；可核对动作由模型输出、被安全层拒绝后位置是否保持安全。

```text
dual_ppo_env.py        联合Gym环境、观测、奖励、PPO运动与安全闸
train_dual_ppo.py      训练、模型保存、配置与训练日志
dual_ppo_demo.py       模型校验、GUI/无窗口演示、报告和A*基线
evaluate_dual_ppo.py   同种子对照、JSON/CSV/Markdown实测数据
plot_dual_ppo_results.py 根据真实日志生成汇报图
test_dual_ppo.py       Gym契约、重置、动作控制权、安全、实模型验收
README_dual_ppo.md     本文
ACCEPTANCE_dual_ppo.md  验收步骤
dual_ppo_artifacts/    本次真实模型与训练/评估记录
```

接口：`reset(seed=...)` 返回 `(obs, info)`；`step([a1,a2])` 返回 `(obs,reward,terminated,truncated,info)`；环境完成后必须 reset；`close()` 断开世界。任务动态提交仍可通过 `env.sim.add_task(bed_id)`，但演示命令只在启动时提交指定任务，尚未针对在线任务流训练。

## 7. 自动验收与边界

```powershell
.\venv\Scripts\python.exe -m unittest test_dual_ppo
.\venv\Scripts\python.exe -m unittest test_complex_ward_demo test_dual_robot_demo test_ppo_patrol
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_ppo_report.json
```

报告校验器要求整批已完成，超时报告被拒绝是预期行为。详细手工/命令行验收见 [ACCEPTANCE_dual_ppo.md](ACCEPTANCE_dual_ppo.md)。

本阶段是二维运动学仿真，没有轮式动力学、真实通信延迟、真实传感器或临床业务。采用单世界顺序提交，R1拥有固定提交优先顺序；策略使用集中观测，部署时需要共享状态。安全层、人员礼让及A*已承担部分协作负担，不能把全部效果归因于强化学习。单次训练与少量测试不能证明任意布局无死锁，也不能证明双机器人一定比单机器人更快。

## 8. 本次实际交付结果

**后续勘误：以下为首轮历史记录。** 首轮随机初始化没有拒绝人员与机器人起点重叠，环形种子3000就存在此问题，因此不能据此归因环形超时或比较算法泛化。现已由 `patrol_scenarios.py` 统一生成无重叠初始场景；新对照全部重新运行，见 [动态规划改进说明](README_adaptive_patrol.md)。原始数据保留，不覆盖历史结果。重新运行旧评估可能与下表不同。

训练种子17，走廊动态环境，实际16,384步，约780秒；模型与日志均已保存。独立评估种子3000—3004，每组5回合，180仿真秒上限，总计40回合：

| 场景 | PPO完成整批 | A*完成整批 | PPO成功平均耗时 | A*成功平均耗时 |
| --- | --- | --- | --- | --- |
| 走廊静态 | 5/5 | 5/5 | 21.06秒 | 21.06秒 |
| 走廊动态 | 5/5 | 5/5 | 28.45秒 | 28.45秒 |
| 环形静态 | 5/5 | 5/5 | 44.92秒 | 44.92秒 |
| 环形动态 | 1/5 | 1/5 | 21.98秒 | 21.98秒 |

所有回合机器人重叠帧均为0。PPO的20回合共发生114帧安全拦截；这不代表发生114次碰撞。环形动态种子3000—3003均超时，PPO分别完成3、0、3、0个任务；基线也未完成整批。不能把这四回合记为成功，不能从成功平均耗时21.98秒推断环形动态比静态更容易。

**本批确定性推理中，两台机器人合计12,154个动作全部为动作1。** PPO参数确实经过梯度训练、推理确实调用模型，但该策略当前主要沿A*路点前进，没有展示主动等待或偏转协作。A*重规划、人员礼让和安全停车承担了避障负担，因此本次只能确认双机器人PPO接入与训练/加载/报告流程已经可复现，不能宣称学到了新的协作避障策略或性能优于A*。后续可针对交叉冲突、会车和通道占用构造训练课程，再做动作多样性及无安全层等单独实验；本次没有将未完成的优化写成已实现能力。

默认动态演示（种子3000）完成4/4任务、75.55仿真秒，两台均完成2个任务，报告校验通过。新增7项测试与前两阶段17项回归测试全部通过，依赖检查无冲突。命令与验证记录见 [validation.md](dual_ppo_artifacts/validation.md)。
