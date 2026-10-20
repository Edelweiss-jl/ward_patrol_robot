# 双机器人巡查：第一步最小版本

本版本对应开发计划的第一步：**两台机器人、共享任务队列、A* 导航、简单分配、安全等待、执行日志**。目标是让两台机器人领取并完成四个普通床位任务，不重复领取，不相互穿过。

原有 `.py`、README 和测试文件均不修改。新入口从 `complex_ward_demo.py` 复用病区布局、模型创建和 A*，因此请保留它与新文件在同一目录。

详细的逐项验收步骤见 [ACCEPTANCE_dual_robot.md](ACCEPTANCE_dual_robot.md)。

## 1. 快速开始

在 PowerShell 中执行：

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
.\venv\Scripts\python.exe dual_robot_demo.py
```

现有 venv 已有 PyBullet；本版本没有引入新的第三方依赖，也不需要安装 PPO、MQTT 或 Web 框架。命令应在项目根目录执行，输出路径相对于当前目录。

PyCharm：解释器选择项目的 `venv\Scripts\python.exe`，运行文件选 `dual_robot_demo.py`，工作目录选择项目根目录。不要选原来的单机器人入口。

启动后两台机器人自动领取任务，无需按开始键。默认保留移动人员；为了先看清任务分配，可运行：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --no-people
```

窗口完成后保留，按 ESC 退出并保存最终报告。按 L 可在运行中保存报告。直接强制结束进程不保证最终汇总报告保存；JSONL 事件则在发生时逐条写入并刷新。

## 2. 默认四个任务和视觉效果

默认场景是六间双床病房的走廊布局。R1 从左侧开始，R2 从右侧开始。

| 任务编号 | 床位 | 内容 |
| --- | --- | --- |
| T001 | N1-1 | 到床位巡检点，停留约 0.6 秒，记录完成 |
| T002 | S3-2 | 同上 |
| T003 | S1-1 | 同上 |
| T004 | N3-2 | 同上 |

任务是四个独立实例，不是四种不同业务。第一轮默认 R1 领取 T001，R2 领取 T002，剩余两项排队；后续归属取决于谁先空闲和当前位置，不硬编码每台一定完成两项。移动人员可能使两台机器人的完成时间不同。

| 画面元素 | 含义 |
| --- | --- |
| 红色机器人、红色路径 | R1 及其本次规划路线 |
| 蓝色机器人、蓝色路径 | R2 及其本次规划路线 |
| 黄色小标记 | 机器人朝向 |
| 橙色人员（环形场景还有推车） | 规则运动的动态障碍 |
| 黄色床位圆点 | 该床位存在未完成任务 |
| 绿色床位圆点 | 该床位提交的任务均完成 |
| 灰色床位圆点 | 没有任务的床位 |
| 紫色方块 | 通过 B 键放置的验收障碍 |

机器人上方显示 ID、状态、任务编号与床位。底部显示完成数量、排队数量、选中的机器人和按键提示。调试文字使用英文，避免 PyBullet 默认字体不支持中文。

截图参数输出的是离屏渲染，包含两台机器人和场景，但不包含 PyBullet 的调试文字与路径线。运行时 GUI 中可以看到这些文字和线条。

## 3. 操作方法

先单击 PyBullet 窗口，让窗口获得键盘焦点。

| 按键 | 操作 | 预期效果 |
| --- | --- | --- |
| 1 / 2 | 选中 R1 / R2 | 底部 SELECT 切换；不改变任务归属 |
| B | 封住选中机器人的当前目标 | 在床位目标处放紫色方块；任务不被错误完成，机器人等待并重试 |
| C | 清除验收障碍 | 自动重规划；任务恢复执行 |
| R | 对两台机器人重新规划 | 从各自当前位置规划到各自目标 |
| 空格 | 暂停 / 继续 | 仿真时间、机器人、人员和任务推进一起暂停 |
| V | 恢复默认相机 | 恢复俯视角 |
| L | 保存当前汇总报告 | 写入 `--report` 指定文件，程序继续运行 |
| ESC | 退出 | 保存汇总报告并释放仿真连接 |

B 是验收工具，不是临床业务。仅允许一个验收障碍存在；选中机器人必须正在执行任务，且目标与机器人/人员距离至少 1.3 米。若拒绝放置，终端输出 `obstacle_rejected`，可以更早操作、换另一台机器人，或先按 C。完成后的场景无需再放障碍，重新运行即可重做演示。

## 4. 调度与导航原理

### 4.1 共享队列

任务状态：`pending → assigned → completed`。

- 按提交顺序扫描任务，为每项任务选择 A* 可行路径最短的空闲机器人；路径等长时按机器人 ID 稳定选择。
- 同一轮分配中，领取任务后立即从空闲集合移除，任务状态立即改为 assigned。
- 每台机器人同一时刻最多一个任务，每个任务同一时刻最多一个执行者。
- 同一床位允许提交多次巡查，但同一时间只分配其中一个，避免两台机器人抢占同一个巡检点。
- 暂时不可达的任务保留队列；当前机器人不可达并不意味着任务已经完成。后面的可达任务可以继续分配。
- 执行中的任务不抢占，不做优先级、等待时间加权，也不自动迁移给另一台机器人。
- 调度周期约 0.65 秒；任务提交和完成会让下一帧立即尝试分配。

这是 **FIFO + A* 路长贪心分配基线**，不声称全局最优，也不强制两台任务数量均分。分配代价忽略瞬时人员和另一台机器人，但考虑家具和临时障碍；实际执行的路径则考虑所有这些对象。

### 4.2 两台机器人如何安全移动

- 共享一个 PyBullet 世界，分别保存位置、路径、当前任务、停留时间、等待状态和统计信息。
- 保留 0.25 米栅格、八邻域 A*、禁止斜穿墙角。机器人半径 0.28 米，额外安全距离 0.16 米。
- 新版本对运动线段与膨胀矩形、人员圆形、另一台机器人圆形做连续几何检查，避免仅检查离散点漏过墙角。
- 每帧依次提交 R1、R2 的移动；R2 检查时能看到 R1 本帧的新位置。移动不能穿过另一台机器人，也不能直接交换位置。
- 无安全路径或下一段受阻时停下，约每 0.65 秒重新尝试规划；恢复移动时记录 `wait_ended`。
- 人员使用规则运动，同时礼让两台机器人；在礼让余量内允许向远离机器人的方向退开，避免原地反复折返。
- 没有任务的机器人尝试返回自己的起点，给床旁留出空间；途中仍可接收任务。所有任务完成后立即结束任务计时、冻结仿真，不强制两台都返航。

这是运动学仿真，通过位置更新控制机器人，并非轮式动力学控制。安全停车不等于保证任何地图都不会死锁：狭窄单通道对向堵塞可能持续等待。本版本未实现通道预约、倒车退让或多机器人全局冲突消解。此类场景应如实报告超时，不能算完成。

## 5. 文件结构与职责

```text
ward_patrol_robot/
├─ complex_ward_demo.py          原有共享场景与 A*，不修改
├─ dynamic_ward_demo.py          原有单病房入口，不修改
├─ multi_room_ward_demo.py       原有多病房入口，不修改
├─ ring_ward_demo.py             原有环形入口，不修改
├─ dual_robot_demo.py           新入口、机器人状态、调度、双机器人控制、日志
├─ test_dual_robot_demo.py       新版自动化仿真验收
├─ verify_dual_report.py         检查导出的完成报告
├─ README_dual_robot.md          本文：使用方法、结构、算法与边界
├─ ACCEPTANCE_dual_robot.md      操作步骤与结果判定
└─ demo_outputs/                运行产生的截图、JSON、JSONL（原项目已忽略此目录）
```

关键对象和方法：

| 对象/方法 | 职责 | 后续扩展位置 |
| --- | --- | --- |
| `PatrolTask` | 任务 ID、床位、目标、归属、状态、创建/分配/完成时间 | 后续加优先级、任务类型 |
| `RobotState` | 每台机器人独立的运行状态和统计量 | PPO 观测与动作接口 |
| `DualRobotSimulation` | 管理两台机器人、共享场景、任务队列 | 保持单一世界，不新建两个 Simulation |
| `add_task(bed_id)` | 验证床位并创建唯一任务编号 | 后续从 MQTT、页面调用 |
| `dispatch()` | 为待处理任务选择空闲机器人 | 替换成优先级/最小总代价匹配 |
| `route()` / `segment()` | A* 寻路 / 连续几何安全检测 | 保留为 PPO 基线与安全检查 |
| `advance_robot()` | 停留、移动、等待及任务完成 | 后续让 PPO 决定局部动作 |
| `emit()` | 统一记录结构化事件 | 后续对接数据库/MQTT |
| `report()` / `save_report()` | 导出当前状态与完整事件历史 | 页面统计和实验对比 |

没有调用旧版单机器人的 `tick()`、巡检 index 或自动逐床推进逻辑。继承只用于复用场景搭建、相机、截图、A* 和基础几何函数。

## 6. 更多运行方式

环形病区默认任务为 ICU1、WARD4、WARD1、ICU4：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --scenario ring
```

自定义普通巡查任务，至少一个，允许多个、允许同床重复巡查：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --beds N1-1 S3-2 S1-1 N3-2 N2-1 S2-2
```

走廊可用床位：N1-1、N1-2、N2-1、N2-2、N3-1、N3-2、S1-1、S1-2、S2-1、S2-2、S3-1、S3-2。

环形可用床位：ICU1、ICU2、ICU3、ICU4、WARD1、WARD2、WARD3、WARD4。名称区分大小写，不能在走廊场景使用 ICU1。

无窗口运行并保存初始场景：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --headless --seconds 240 --screenshot demo_outputs/dual_robot.png
.\venv\Scripts\python.exe verify_dual_report.py
```

自动放障碍及恢复：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --headless --block-test --report demo_outputs/dual_block_report.json --events demo_outputs/dual_block_events.jsonl
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_block_report.json
```

`--block-test` 在约第 1 个仿真秒封住 R1 的目标，第 5 秒清除。默认四任务场景可直接使用；若自定义任务导致目标离机器人太近，工具会拒绝插入并报错，避免强行把障碍生成在机器人身上。

`--seconds` 只限制无窗口运行的**仿真时间**，不是电脑实际用时。无窗口模式全部完成返回 0，超时未完成返回 1，参数错误返回 2，Ctrl+C 返回 130。GUI 按 ESC 正常关闭返回 0，是否完成应查看报告，不能仅看退出码。

默认输出：`demo_outputs/dual_robot_report.json` 和 `demo_outputs/dual_robot_events.jsonl`。再次运行会覆盖同名输出；保留不同实验时显式指定 `--report`、`--events`。

## 7. 日志与结果解释

JSONL 每行一条事件；JSON 汇总包含 `tasks`、`robots`、`events` 和统计信息。所有时间均为仿真秒，不是墙上时钟。任务 ID 在每次启动时从 T001 重新计数。

事件类型：

- `task_created`：任务进入队列。
- `task_assigned`：任务分给机器人，含估算路径长度。
- `path_planned`：规划是否有路、路点数量。
- `wait_started` / `wait_ended`：受阻与恢复。
- `task_arrived`：进入床位到达范围。
- `task_completed`：完成约 0.6 秒停留后记录完成。
- `all_tasks_completed`：整批结束。
- `obstacle_added` / `obstacles_cleared` / `obstacle_rejected`：验收障碍操作。
- `paused` / `resumed` / `simulation_started` / `simulation_stopped`：生命周期。

每条事件有 `event_id`、`event`、`sim_time`；任务相关事件有 `task_id`、`bed_id`，机器人相关事件有 `robot_id`。返航规划、无任务等待不关联任务编号。

| 字段 | 解释 |
| --- | --- |
| `completed` | 是否整批任务完成 |
| `completed_tasks` / `task_count` | 已完成与总任务数 |
| `parallel_task_seconds` | 两台机器人同时持有任务的时间，可能包含等待 |
| `simultaneous_motion_seconds` | 两台机器人同一帧都移动的累计时间，可能包含返航 |
| `min_robot_center_distance_m` | 每帧记录的机器人中心最小距离 |
| `robot_overlap_frames` | 中心距离不大于两倍半径的帧数，应为 0；不代表所有碰撞类型 |
| `robots[].distance_m` | 包含返航的累计移动距离 |
| `robots[].wait_seconds` | 包含返航等待的累计受阻时间 |
| `tasks[].assigned_at - created_at` | 排队等待时间 |
| `tasks[].completed_at - assigned_at` | 任务执行时间，含途中等待和到床停留 |

报告检验只能核对记录，不能代替运行时逐帧几何与 PyBullet 碰撞测试。请一起运行自动化测试。

## 8. 已实现范围和下一阶段

本版本实现普通巡查任务的双机器人最小流程；任务数可以超过两台机器人的数量。尚未实现 PPO、MQTT、虚拟体征、SQLite、Web、异常复查业务、呼叫响应业务、优先级加权、任务超时自动改派和通道预约。

下一步可以先在单机器人训练环境验证 PPO，再将策略接入 `advance_robot()` 的动作决策；`dispatch()` 继续负责分配。不要把任务分配、局部避障、业务告警合并成一个训练问题。

测试通过的范围是本文及验收指南列出的场景，不等于任意布局、任意数量任务都能无死锁完成，也不证明双机器人一定比单机器人快。
