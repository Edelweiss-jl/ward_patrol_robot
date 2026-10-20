# 第三阶段改进：动态路径规划与 Q-learning 自适应选择

目标是在相同安全约束下缩短巡查时间、提高完成率。保留原A*和PPO的源码、模型及历史结果，新增动态规划器、真实训练的Q-learning模式选择器和配对实验。最终成绩以本文实测章节和原始评估数据为准，不保证每个场景都比A*快。

## 1. 运行

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
.\venv\Scripts\python.exe adaptive_patrol_demo.py
```

默认两台机器人、动态走廊、四个任务、种子4000，加载 `adaptive_artifacts/q_policy.json`，无需重训。空格同时暂停/继续人员和机器人，ESC退出；完成或超时自动关闭窗口并保存报告。PyCharm选本文件、项目venv解释器和项目根工作目录。

```powershell
# 动态环形、无窗口及自定义任务
.\venv\Scripts\python.exe adaptive_patrol_demo.py --scenario ring --seed 4005
.\venv\Scripts\python.exe adaptive_patrol_demo.py --headless --report demo_outputs/adaptive_report.json
.\venv\Scripts\python.exe adaptive_patrol_demo.py --headless --beds N1-1 S3-2 N2-1 S2-2 --seconds 240

# 相同初始条件的原A*和两种禁用学习的消融对照
.\venv\Scripts\python.exe adaptive_patrol_demo.py --controller astar --headless --report demo_outputs/adaptive_astar.json
.\venv\Scripts\python.exe adaptive_patrol_demo.py --controller fixed --headless --report demo_outputs/adaptive_fixed.json
.\venv\Scripts\python.exe adaptive_patrol_demo.py --controller short --headless --report demo_outputs/adaptive_short.json
```

`--no-people`可关闭人员。任务床号同第一阶段，走廊N1-1到S3-2，环形ICU1—4、WARD1—4，名称区分大小写；允许重复任务，同床串行。退出码：全部完成0，超时/ESC中断1，策略或参数错误2。策略缺失、版本错误、训练未完成会明确报错，不静默降级。默认报告覆盖同名文件，保留实验时指定不同路径。`--seconds`默认180，是仿真秒。

## 2. 为什么不只换神经网络算法

上一版PPO确定性测试始终选择前进，仅换成DQN、SAC或增加训练步数不能解决“动态障碍让全局路径变空”和“栅格路线拐折多”。本版拆开职责：A*提供静态全局路线，局部规划根据当前人员位置及估计速度动态绕障，Q-learning按现场状态选择规划模式。

这是 **Q-learning + 平滑A* + 预测局部规划 + 安全停车** 的混合系统。强化学习没有替代A*；固定模式对照用于辨别规划本身和学习选择各自的收益。

## 3. 动态规划流程与公平约束

1. 任务分配仍用原FIFO和静态A*路长，保留任务唯一领取和同床串行规则。
2. 切换目标时先规划静态全局骨架，不让目标旁人员导致整条路线消失。
3. 用连续几何检查做可见性平滑，只有不穿墙和家具的折点才可跳过。
4. 每0.2秒分别选择两台机器人的局部规划模式。
5. 在参考方向附近采样12个方向：0、±20、±40、±65、±90、±125、180度。检查线段安全，预测人员相对扫掠距离，按目标进展、偏转和间距评分。
6. 候选终点必须仍可静态连回下一全局路点，防止绕障后进入错误病房；失去可见连接时重新规划。
7. 每1/60秒检查实际移动，危险移动只停车；每0.2秒重新观察和选择方向，随动态障碍改变行走路径。

预测速度来自上一0.2秒实际位移，首周期为0，不读取人员未来真实轨迹。人员仍同时礼让两台机器人；机器人顺序提交位置，禁止相互穿过。预测可有误差，安全闸仍需工作。

两方案速度均0.95米/秒、半径0.28米、安全余量0.16米、到达阈值0.025米、床旁停留0.6秒，任务调度和人员规则相同。提速不能来自提高速度、减小半径或减少停留。

## 4. 强化学习：状态、动作与更新

使用共享表格Q-learning，而非神经网络；36种状态可完整导出，适合检查学习行为。两台机器人共享Q表，分别选择模式。

| 状态因素 | 分箱 |
| --- | --- |
| 最近人员到机器人表面的间距 | >1.5米、0.6—1.5米、≤0.6米 |
| 另一台机器人中心距离 | >3米、1.2—3米、≤1.2米 |
| 是否超过1秒未移动 | 否/是 |
| 距当前目标是否小于1.2米 | 否/是 |

合计3×3×2×2=36，没有床号、种子、测试标签或未来任务信息。

| 动作 | 模式 | 前视距离 | 行为 |
| --- | --- | --- | --- |
| 0 | short | 0.45米 | 0.65秒预测，较短前视 |
| 1 | cruise | 1.4米 | 1秒预测，沿平滑路线推进 |
| 2 | detour | 2米 | 1秒预测，提高候选评分的间距权重 |
| 3 | yield | 停车 | 等待一个0.2秒周期 |

Q-learning决定规划模式，局部规划器选择具体方向。候选均不安全时停车，这是明确的安全机制，不是假装模型选了等待。

```text
Q(s,a) <- Q(s,a) + 0.12 * [r + 0.97 * max Q(s',a') - Q(s,a)]
回合结束时未来价值为0。
每台每周期 r = 4 × 到当前全局路点的距离减少量 - 0.04 + 15 × 新完成任务数
```

反向运动产生负进展奖励；等待有时间成本。初始Q全0，epsilon-greedy探索，训练时随机打破并列值。没有专家标签、监督模仿或前进价值预置。状态离散化与模式设计属于工程选择，有限训练不构成最优性证明。

## 5. 训练与复现

```powershell
.\venv\Scripts\python.exe train_adaptive_patrol.py --episodes 60 --seed 23 --scenario-seed 1000 --output adaptive_artifacts_new
```

输出目录必须为空或不存在，不覆盖交付。每回合重新创建实际PyBullet双机器人病区；走廊/环形交替、有人员、场景种子1000—1059，任务顺序、合法人员位置和速度随机化。上限120仿真秒；探索率0.45线性降至0.05，alpha0.12、gamma0.97。

`q_policy.json`保存完整Q值、访问次数、参数、种子、实际回合/转移数、墙钟时间和完成标记；`training.json`保存逐回合奖励、耗时、成功和探索率。中断仍保存已有数据并标记未完成，演示拒绝加载未完成策略。依赖复用PyBullet和NumPy，无新增框架；完整环境仍可用 `requirements_ppo.txt` 恢复。

## 6. 配对评估与正确解读

```powershell
# 动态：20种子×2布局×4控制器=160回合
.\venv\Scripts\python.exe evaluate_adaptive_patrol.py --episodes 20 --seed 4000 --controllers astar fixed short adaptive

# 静态补充，另存
.\venv\Scripts\python.exe evaluate_adaptive_patrol.py --episodes 5 --seed 5000 --static --controllers astar fixed short adaptive --output adaptive_artifacts/evaluation_static.json
```

训练1000—1059、开发3000附近、正式动态测试4000—4019分开。控制器均重新建世界，初始任务顺序、人员位置/速度、机器人起点完全相同；初始条件及SHA256写入记录，配对不一致即报错。按种子交替控制器执行顺序，减小系统负载偏差。

| 指标 | 解释 |
| --- | --- |
| 完成率 | 时限内完成整批，部分完成也算失败 |
| 配对提速 | 仅双方都成功的同种子回合，对 `(A*时间-本方法时间)/A*时间` 求平均 |
| 更快数量 | 严格更快回合数/双方均成功配对数 |
| 成功平均时间 | 仅成功回合，必须结合完成率解释 |
| PAR2 | 成功用实际时间，失败用2×时间上限，对全部回合平均，越低越好 |
| 碰撞帧 | 分别审计墙体、人员、机器人接触，正常应为0 |
| 电脑实际秒数 | 包含建场景、规划、决策和仿真，受机器负载影响 |

任务仿真耗时是主要效率指标，单次墙钟采样不是实时性能上界。人员仍礼让机器人，后续人员轨迹随机器人行为变化，但交互规则相同。`fixed`固定cruise，`short`固定short，均保留新规划器而禁用学习；固定模式也有的收益不能全部写成强化学习收益。

## 7. 上一版评估勘误

旧随机初始化未排除人员与机器人起点重叠。例如旧环形种子3000中R1为(-9.5,0)，人员为(-9.4,0.5144)，中心距离约0.524米，小于半径之和0.54米。因此旧环形动态超时不能全部归因于算法。

现在 `patrol_scenarios.py` 统一拒绝机器人/人员、人员/人员及静态障碍初始冲突，并用于原PPO环境和新评估。旧数据保留且注明勘误；新A*基线全部重新跑，没有把修复场景生成器的收益计成新算法收益。

## 8. 文件、报告与验收

```text
adaptive_patrol.py          平滑全局路线、预测局部规划、安全闸和模式状态
adaptive_patrol_demo.py     演示、Q表加载、配对初始条件及逐帧审计
train_adaptive_patrol.py    真实共享Q-learning训练
evaluate_adaptive_patrol.py 配对JSON/CSV/Markdown评估
patrol_scenarios.py         共享的无初始冲突场景生成器
test_adaptive_patrol.py     初始化、公平性、安全、停车、限速及实策略验收
adaptive_artifacts/        策略、训练记录、评估和验证说明
```

报告保留任务事件、距离/等待和并行统计，增加 `mode_counts`、状态/模式轨迹、全局规划次数/用时、初始条件、墙体/人员碰撞帧和最小人员表面间距。`navigation_debug`记录结束时路径、目标和人员位置。演示每0.2秒保存轨迹，批评估只存逐回合指标和初始条件。

```powershell
.\venv\Scripts\python.exe -m unittest test_adaptive_patrol
.\venv\Scripts\python.exe -m unittest test_dual_ppo test_complex_ward_demo test_dual_robot_demo test_ppo_patrol
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/adaptive_report.json
```

## 9. 局限

36状态共享表尚未覆盖任意多机器人、陌生建筑、真实传感器噪声及在线任务流。预测是常速度近似，仍可能停车、绕远或超时；没有完备通道预约和全局死锁消解。仍是二维运动学仿真，不是临床或真实机器人部署验证。整体均值提升不代表每个回合都更快，必须保留退化和失败记录。
