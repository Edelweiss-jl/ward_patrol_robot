# 第二步：单机器人 PPO 最小验证

本版本实现：**单机器人简单病房训练环境 → 真正的 SB3 PPO 训练 → 保存/加载模型 → A* 路点引导下的局部动作 → 固定种子对照评估**。

本阶段没有把 PPO 接进双机器人调度，那是第三步。原有单机器人和双机器人代码、README 均不修改；只新增文件，并在项目现有 venv 中安装训练依赖。

操作与结果验收见 [ACCEPTANCE_ppo.md](ACCEPTANCE_ppo.md)。本次实际评估数据见 [ppo_artifacts/evaluation.md](ppo_artifacts/evaluation.md)，逐回合原始记录保存在同目录的 JSON 和 CSV 中。

## 1. 直接运行交付的模型

PowerShell：

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
.\venv\Scripts\python.exe ppo_patrol_demo.py
```

这会加载 `ppo_artifacts/ppo_patrol.zip`，运行动态场景、种子 1000。无需重新训练。

画面中：红色圆柱是机器人，橙色圆柱是移动人员，蓝色家具包含中央病床与两侧柜子，绿色圆点是目标；绿色折线是 A* 全局路径。机器人局部动作由 PPO 选择，A* 折线本身不会驱动机器人移动。

这是独立的简化 8×6 米训练场景，不是原来的六病房或环形病区。它用于先验证模型训练和推理流程，不能把这些实验结果写成已经在复杂病区/双机器人上验证。

GUI 操作：

| 操作 | 效果 |
| --- | --- |
| 启动程序 | 自动加载模型并开始一个回合 |
| 单击窗口后按空格 | 暂停/继续；人员和机器人一起暂停 |
| ESC | 退出，保存本回合结果及动作轨迹 |
| 回合成功、碰撞或超时 | 停止推进，终端输出结果，窗口保留供查看；按 ESC 退出 |

程序默认执行一个回合，不会无限重启。要看另一个场景，退出后用不同的 `--seed` 重新运行。失败不是程序崩溃，失败回合同样保留报告。

PyCharm：选择项目 `venv\Scripts\python.exe`，运行文件选择 `ppo_patrol_demo.py`，工作目录选项目根目录。模型路径相对于工作目录，路径错误时应修正工作目录或显式传入 `--model`。

## 2. 无窗口运行与对照

```powershell
# PPO 动态场景
.\venv\Scripts\python.exe ppo_patrol_demo.py --headless --seed 1000 --screenshot demo_outputs/ppo_initial.png

# 同一个场景的 A* + 一步安全检查对照
.\venv\Scripts\python.exe ppo_patrol_demo.py --controller astar --headless --seed 1000 --report demo_outputs/ppo_astar_demo.json

# 静态场景，检查路点跟随
.\venv\Scripts\python.exe ppo_patrol_demo.py --mode static --headless --seed 1000 --report demo_outputs/ppo_static_demo.json
```

默认报告路径是 `demo_outputs/ppo_demo_report.json`。报告有 `controller`、`mode`、`seed`、`outcome`、`success`、`collision`、仿真耗时、距离以及逐步 `trajectory`（动作、坐标、奖励）。重复运行同名路径会覆盖，请用 `--report` 保留不同实验。

退出码：成功 0；碰撞、超时或提前中断 1；参数错误 2。默认模型缺失会报错，不会自动改用规则控制器。

## 3. A* 与 PPO 分别负责什么

### A*：全局路线与前视目标

- 每个回合重置时，基于静态家具和墙体生成 0.25 米栅格路径。
- 规划时额外保留 0.08 米余量；机器人半径为 0.22 米。
- 根据机器人在路径上的投影，选取前方约 0.65 米的目标点。
- 如果前视线段穿过床角，退回可见的路径点。
- A* 不随移动人员每步重规划，也不替 PPO 选择避障动作。

### PPO：选择局部动作

采用 SB3 `PPO("MlpPolicy", ...)`，输入 28 维观测，输出 9 个离散动作之一：

| 动作编号 | 含义 |
| --- | --- |
| 0 | 等待一个控制周期 |
| 1 | 朝 A* 前视目标方向移动 |
| 2—8 | 将该方向分别旋转 45、90、135、180、225、270、315 度后移动 |

每步 0.2 仿真秒，速度 0.8 米/秒，即非等待动作移动 0.16 米。控制是二维运动学，不含轮式动力学、转向惯性或真实传感器。

**动作 1 的参考方向来自 A*，但是否前进、等待或偏转由模型决定。** 没有隐藏的 A* 兜底、强制安全动作替换或对专家动作的监督学习。

观测包括：

- A* 前视目标方向：2 维。
- 最终目标相对偏移：2 维。
- 机器人位置：2 维。
- 人员相对位置、速度：4 维。
- 是否有动态人员：1 维。
- 8 束 PyBullet 理想射线距离：8 维。
- 9 种动作未来 0.6 秒的碰撞风险标志：9 维（安全 +1、有风险 -1）。这是根据地图和已知人员运动模型计算的特征，不是动作掩码，模型仍可选择有风险动作并失败。

观测归一化为 [-1,1]，使用仿真真值与理想射线，无定位误差或传感噪声。静态场景中的人员位置/速度观测置零。短时风险特征依赖已知人员轨迹，这降低了学习难度；真实部署需要预测人员运动并处理预测误差。

### 奖励与结束条件

路径剩余量 = 全局路径剩余弧长 + 机器人偏离路径的距离。

```text
普通一步奖励 = 3 × 路径剩余量减少值 - 0.015 - 0.6 × 是否选择有短时风险的动作
到达目标（距离 < 0.25 米）额外 +15
碰撞动作尝试奖励 -30，并立即结束回合
180 步仍未到达（36 仿真秒）超时，额外 -3
```

几何检查使用运动线段与膨胀家具矩形，以及机器人与人员的相对运动线段。若动作会碰撞，机器人停在上一个安全位置，但该回合**已判定碰撞失败**；不是把它修正成安全动作继续成功。人物不会礼让机器人，按照种子确定的正弦路线持续移动。

机器人在 PyBullet 中是视觉模型，碰撞结果由解析几何检查计算，不依赖刚体接触求解。人员在一个控制周期内的扫掠用起终点线段近似。不能将本项目碰撞率等同于真实机器人的接触事故率。

## 4. 重新训练

训练需要 CPU 时间，已有模型可直接演示，不必每次运行都训练。

```powershell
.\venv\Scripts\python.exe train_ppo_patrol.py --steps 40000 --seed 7 --output ppo_artifacts_new
```

前约 1/8 步数在静态场景预热，其余在动态场景继续训练。两个阶段使用同一 PPO 模型。`--steps` 是请求步数，实际按 512 步 rollout 取整，因此可能略多，记录在 `training_config.json`。

已有同名模型时脚本拒绝覆盖；想保留本次交付，请使用新 `--output`。确认需要替换时可显式传 `--overwrite`。Ctrl+C 会尽量保存 `ppo_interrupted.zip` 并记录 `completed=false`，它不等于完整训练模型。

固定训练设置：CPU、单个 PyTorch 线程、两层 64 单元 MLP、rollout 512、batch 64、6 个优化 epoch、学习率 0.0003、gamma 0.99、GAE lambda 0.95、entropy coefficient 0.015。随机种子、库版本、实际训练步数及墙钟耗时均记录在配置中。

训练文件：

| 文件 | 用途 |
| --- | --- |
| ppo_patrol.zip | 完整静态预热＋动态训练后的模型，演示默认加载 |
| ppo_static.zip | 静态预热完成时的模型，便于对比 |
| training_config.json | 配置、版本、种子、实际步数、是否完成 |
| static_monitor.csv | 静态训练每回合奖励和长度 |
| dynamic_monitor.csv | 动态训练每回合奖励和长度 |
| progress.csv | PPO 优化过程日志，含损失、熵、训练步数等 |

训练奖励不是成功率，必须另跑评估。不同库版本、CPU 环境可能产生数值差异；固定种子不能保证跨机器逐字节相同模型。

## 5. 批量评估与结果解读

```powershell
.\venv\Scripts\python.exe evaluate_ppo_patrol.py --model ppo_artifacts/ppo_patrol.zip --episodes 20 --seed 2000 --output ppo_artifacts/evaluation.json
```

该命令对静态、动态两种模式，分别运行 PPO、A* 规则基线、随机策略，每组 20 回合，总共 120 回合。每组使用种子 2000—2019，和训练种子分开。PPO 采用确定性推理，不在评估时更新参数。

输出：JSON（摘要和原始回合）、CSV（逐回合）、Markdown（可阅读的摘要），并记录模型 SHA256。

| 指标 | 解释 |
| --- | --- |
| success_rate | 成功到达回合数 / 总回合数 |
| collision_rate | 碰撞动作尝试导致失败的回合数 / 总回合数 |
| timeout_rate | 180 步未完成且未碰撞的回合数 / 总回合数 |
| mean_episode_seconds | 全部回合平均仿真时间，包括早期碰撞失败 |
| mean_success_seconds | 仅成功回合的平均仿真时间；没有成功时为 null |
| mean_success_distance_m | 仅成功回合的平均移动距离 |
| wall_seconds | 原始记录中的实际运行用时，受机器与系统负载影响 |
| action_counts | 每回合 9 类动作次数，检查模型实际选择了什么 |

三个比例应相加为 1。较短的全部回合耗时可能只是更早撞障，因此不能单独作为性能改善证据。

A* 规则基线会检查下一步候选动作，并知道人员下一控制周期的位置；从安全动作中选择剩余路径量最小的一个。它是理想信息下的规则基线，不是纯静态 A*。PPO 的风险观测也使用已知轨迹，预测窗口是 0.6 秒；基线只检查下一步 0.2 秒，因此不是等价预测窗口下的算法消融实验。PPO 若未超过它，应如实记录，不能修改评估口径掩盖。

当前评估属于小样本最小验证，不证明复杂病区泛化，也不证明 PPO 优于所有传统方法。

## 6. 文件结构和后续接口

```text
ppo_patrol_env.py       Gymnasium 环境、A*、几何检测、观测与奖励、PyBullet 场景
train_ppo_patrol.py     训练与保存模型
evaluate_ppo_patrol.py  相同场景种子的对照评估
plot_ppo_results.py     根据真实日志生成训练与评估图
ppo_patrol_demo.py      加载模型后 GUI/无窗口演示
test_ppo_patrol.py      环境与模型验收测试
requirements_ppo.txt    第二阶段新增依赖版本
README_ppo.md           本说明
ACCEPTANCE_ppo.md       验收操作与预期结果
ppo_artifacts/         本次交付模型、训练日志、评估原始数据
```

第三步接入双机器人时，需要让每台机器人构造同样语义的观测，并加入另一台机器人的信息后重新训练/验证。不能把当前“单个移动人员”的输入直接当成任意多机器人观测。

当前 `step(action)` 已把观测、动作、奖励和结束条件明确分开，为后续接入提供基础。任务调度继续由第一步的调度器负责，PPO 负责局部动作。

## 7. 环境依赖和恢复

本机现有 Python 3.10 venv 已安装并验证依赖。如果换电脑或创建新环境，可执行：

```powershell
.\venv\Scripts\python.exe -m pip install -r requirements_ppo.txt --extra-index-url https://download.pytorch.org/whl/cpu
.\venv\Scripts\python.exe -m pip check
```

PyTorch CPU wheel 较大，需要联网。不要为了演示再重复安装或升级。其他传递依赖的本次实际版本可参考交付的 `ppo_artifacts/environment_freeze.txt`。

接口设计参考：[SB3 自定义环境文档](https://stable-baselines3.readthedocs.io/en/v2.4.1/guide/custom_env.html)。本项目实际固定 SB3 2.4.1 与 Gymnasium 1.0.0，并通过 `check_env` 检查。

## 8. 汇报图片

交付的 [结果图](ppo_artifacts/results.png) 同时展示分组评估和训练奖励变化。需要在重新评估后更新图片：

```powershell
.\venv\Scripts\python.exe plot_ppo_results.py
```

训练图中的虚线是从静态场景切换到动态场景的位置；奖励下降不等于代码故障，也不能仅凭奖励上升证明避障已经可靠。

## 9. 本次交付的实测结果

交付模型版本为 `ward-local-v2`，28 维观测；请求训练 40,000 步，实际完成 40,448 步（静态预热 5,120 步），本次训练约 206 秒。训练种子为 7。最终评估使用独立场景种子 2000—2019：

| 场景 | PPO 成功 / 碰撞 / 超时 | A* 规则基线成功 / 碰撞 / 超时 | PPO 成功回合平均时间 | A* 成功回合平均时间 |
| --- | --- | --- | --- | --- |
| 静态 | 20 / 0 / 0 | 20 / 0 / 0 | 8.74 仿真秒 | 8.68 仿真秒 |
| 动态 | 19 / 1 / 0 | 19 / 1 / 0 | 9.61 仿真秒 | 9.12 仿真秒 |

结论：已经完成真实模型训练、加载、局部动作控制和对照记录；这批动态场景中 PPO 成功率为 95%，与规则基线相同，耗时略长，不能宣称 PPO 更优。仍有一个碰撞失败回合，尚不适合无安全协调地接入双机器人。

首轮 `ward-local-v1` 的模型、日志和环境源码保留在 `ppo_artifacts_v1/`。首轮动态成功率为 50%，动作只会向前，之后增加短时风险特征和风险奖励重新训练。首轮与最终轮使用不同测试种子，因此不是严格的单变量性能提升实验；归档用于说明问题与修正过程。v1 的 19 维模型不能加载到现有 28 维环境。
