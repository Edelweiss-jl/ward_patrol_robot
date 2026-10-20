# 双机器人最小版本验收指南

验收目标：两台机器人通过同一队列领取至少四项普通床位巡查任务，能够同时工作，任务不重复执行，受阻后安全等待，清障后继续，全过程可查询。

这是第一阶段验收，不要求 PPO、MQTT、异常复查或 Web。详细结构见 [README_dual_robot.md](README_dual_robot.md)。

## 1. 验收准备

PowerShell 进入项目：

```powershell
Set-Location 'E:\python汇报\ward_patrol_robot'
```

以下命令使用现有 venv。不要同时在同一个 Python 进程创建多个仿真实例。退出当前 GUI 后再开始下一个人工演示。

## 2. 人工验收 A：两台机器人领取四个任务

操作：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py
```

| 步骤 | 操作方法 | 应看到的结果 / 成效 |
| --- | --- | --- |
| 1 | 启动后观察 | 红色 R1 位于左侧，蓝色 R2 位于右侧，处于同一个病区 |
| 2 | 查看终端前几条事件 | 四个 task_created；R1 领取 T001，R2 领取 T002；T003、T004 排队 |
| 3 | 观察两台机器人移动 | 两台各沿自己的路径前往目标；并非复制同一台机器人的动作 |
| 4 | 等待第一项任务完成 | 到床停留约 0.6 秒，终端出现 task_completed；空闲机器人继续领队列任务 |
| 5 | 等待整批结束 | 底部 COMPLETE、DONE 4/4、QUEUED 0；四个目标圆点变绿 |
| 6 | 按 ESC | 退出并生成 JSON 汇总和 JSONL 事件日志 |

任务最终归属可能不均分，验收不要求每台恰好两项；要求两台均有完成任务且不存在重复领取。

结果核对：

```powershell
.\venv\Scripts\python.exe verify_dual_report.py
```

通过标准：输出 `PASS`，退出码为 0。该脚本要求至少四项任务，所以一项任务的自定义小演示即使完成，也不满足这组验收标准。

## 3. 人工验收 B：安全等待和恢复

操作：重新运行默认入口，启动后的前 1—2 秒点击窗口，按 1，再按 B。想更容易操作，可加 `--no-people`。

| 步骤 | 操作 | 预期结果 |
| --- | --- | --- |
| 1 | 按 1、B | R1 的目标处出现紫色方块；出现 obstacle_added |
| 2 | 等待几秒 | R1 显示 WAITING；对应任务保持 assigned，不会计为完成；日志出现 wait_started |
| 3 | 观察 R2 | R2 可继续执行自己的任务；若也受阻，则同样安全等待 |
| 4 | 按 C | 方块消失，重新规划，R1 恢复移动，日志出现 wait_ended |
| 5 | 等整批完成、按 ESC | 最终仍是 4/4，不重复创建或重复完成同一任务 |

如果 B 被拒绝，查看终端 `obstacle_rejected` 原因：机器人可能已接近目标、已完成任务，或已有一个障碍。不要把“拒绝放置”误当成“障碍已生效”。可重新开始，或选中另一台尚在执行的机器人再操作。

不想手动按键时，可运行确定性演示（GUI 或无窗口都支持）：

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --block-test
```

此模式在约 1 秒放障碍、5 秒清障。以上均为仿真秒，电脑实际运行时间可能更长。

## 4. 人工验收 C：暂停与状态显示

1. 运行期间按空格：两台机器人与人员都应停止，底部显示 PAUSED。
2. 等待几秒后再按空格：继续原任务，不能重新分配已执行中的任务。
3. 按 2：底部 SELECT R2，只改变操作对象，不改变任务归属。
4. 按 R：重新规划后继续执行，不清空任务队列。
5. 按 V：恢复默认观察视角。
6. 按 L：生成当前汇总；中途保存显示 completed=false 是正常的，不应拿它当完成报告。

## 5. 自动验收：完整运行与报告检查

### A. 默认走廊场景

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --headless --seconds 240 --report demo_outputs/dual_robot_report.json --events demo_outputs/dual_robot_events.jsonl --screenshot demo_outputs/dual_robot.png
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_robot_report.json
```

### B. 环形场景

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --scenario ring --headless --seconds 240 --report demo_outputs/dual_ring_report.json --events demo_outputs/dual_ring_events.jsonl
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_ring_report.json
```

### C. 自动障碍恢复场景

```powershell
.\venv\Scripts\python.exe dual_robot_demo.py --headless --block-test --report demo_outputs/dual_block_report.json --events demo_outputs/dual_block_events.jsonl
.\venv\Scripts\python.exe verify_dual_report.py demo_outputs/dual_block_report.json
```

每个运行命令都应返回 0，随后报告检查应 PASS。PowerShell 可在命令执行后输入 `$LASTEXITCODE` 查看该命令退出码；不要只看终端有没有异常文本。

### D. 新旧功能自动化测试

```powershell
.\venv\Scripts\python.exe -m unittest test_dual_robot_demo test_complex_ward_demo -v
```

预期：`Ran 10 tests`，最后为 `OK`。包含八项新增测试和两项旧版回归测试；部分测试内部会分别检查两种地图。完整测试可能需要几十秒或更久，取决于电脑性能。

| 新增测试 | 验收内容 |
| --- | --- |
| test_default_patrol_both_layouts | 两种布局完成四项任务；两台都参与；创建/领取/到达/完成均恰好一次；逐帧几何安全，定期检查 PyBullet 接触 |
| test_blocked_goal_wait_clear_and_resume | 目标被封时不移动、不误报完成；清除后自动恢复 |
| test_head_on_motion_cannot_cross | 人工设置两台机器人对向移动，安全闸阻止穿过或位置交换 |
| test_pause_freezes_world_and_events | 暂停时仿真时间、人员、机器人及事件数不推进 |
| test_dynamic_submission_and_same_bed_serialization | 运行中提交任务、同床任务串行、完成后再提交、拒绝未知床位 |
| test_precise_corner_check_and_report_log | 墙角线段不能漏检；JSONL 与 JSON 事件一致 |
| test_unreachable_pending_task_is_not_assigned | 无路的待处理任务保留 pending，其他可达任务仍可分配 |
| test_report_verifier_rejects_incomplete_run | 未完成报告不能误判为 PASS |

对向测试验证的是**安全停止**，不声称已经解决窄通道死锁。它会故意关闭重规划以隔离安全检查，不应拿该测试证明完整避让策略的有效性。

## 6. 如何查看结果

```powershell
$dualResult = Get-Content demo_outputs/dual_robot_report.json -Raw | ConvertFrom-Json
$dualResult.tasks | Format-Table task_id, bed_id, status, robot_id, assigned_at, completed_at
$dualResult.robots | Format-Table robot_id, completed_tasks, distance_m, wait_seconds
$dualResult | Select-Object completed, completed_tasks, task_count, simulated_seconds, simultaneous_motion_seconds, min_robot_center_distance_m, robot_overlap_frames
```

验收必须满足：

| 字段 / 证据 | 通过标准 | 体现的成效 |
| --- | --- | --- |
| completed | true | 整批完成 |
| completed_tasks / task_count | 默认演示均为 4 | 没有漏任务 |
| tasks 中的 task_id | 唯一 | 任务实例不会混淆 |
| 每项 task_assigned、task_completed | 各一条，robot_id 一致 | 不重复领取、不重复完成 |
| robots[].completed_tasks | 两台均大于 0 | 两台都实际参与执行 |
| simultaneous_motion_seconds | 大于 0 | 两台有同时移动，而非完全串行 |
| robot_overlap_frames | 0 | 记录中没有机器人重叠 |
| min_robot_center_distance_m | 大于 0.56 米 | 机器人本体不接触；规划还保留额外余量 |
| 自动测试结果 | OK | 对家具、人员及机器人接触进行了额外检查 |

当前安全统计只有机器人对机器人重叠帧数，不能将它描述为“全场景所有碰撞计数”。家具和人员安全由自动测试中的几何/PyBullet 检查补充验证。

## 7. 开发验收记录与边界

2026-10-19 已在项目现有虚拟环境执行无窗口仿真、报告核对与自动测试。已检查双机器人初始场景的离屏图片。GUI 的实际按键、窗口文字阅读和手动演示按本指南由使用者操作验收；不将离屏图片检查视为已完成 GUI 按键验收。

本次实际生成的报告结果（均开启默认移动人员）：

| 场景 | 完成任务 | 仿真总时间 | 两台同时移动时间 | 最小机器人中心距离 | 重叠帧数 |
| --- | --- | --- | --- | --- | --- |
| 默认走廊 | 4/4 | 105.800 秒 | 67.467 秒 | 1.9683 米 | 0 |
| 环形病区 | 4/4 | 34.283 秒 | 29.933 秒 | 4.5000 米 | 0 |
| 走廊自动障碍恢复 | 4/4 | 65.733 秒 | 41.467 秒 | 1.9683 米 | 0 |

三份报告均通过 `verify_dual_report.py`。障碍恢复场景耗时比默认场景短，是因为放置障碍改变了任务分配与人员相遇时序，不能解释为“加障碍提高效率”。这些是单次确定性演示结果，不是性能优化结论。代码、参数或运行环境改变后应重新记录。

新旧合并测试实际结果：`Ran 10 tests in 63.784s`，`OK`。完整终端记录保存于 `demo_outputs/dual_acceptance_tests.txt`。

另外检查了无窗口超时退出码 1、非法床位参数退出码 2，以及未完成报告被验收脚本拒绝的情况。

本版新增独立文件，原有演示和测试不修改。完成标准仅覆盖普通巡查与基础分配，不包括优先级、PPO、MQTT、SQLite、Web 和医学判断。

若自定义任务在窄通道持续互堵，程序可能等待到超时，报告应保持未完成。这属于当前基础协调能力的边界，后续应增加通道预约/退让，不能通过穿过另一台机器人来“解决”。

## 8. 汇报建议

可以展示：默认运行截图、四项任务分配/完成日志、障碍恢复过程、测试结果，以及两个场景的仿真耗时。

建议表述：

> 已在同一 PyBullet 病区内实现两台机器人，共享普通巡查任务队列；使用基于 A* 路长的贪心分配和逐步安全检查，支持并行执行、受阻等待及清障恢复。任务归属、到达和完成过程可导出查询。下一步将验证 PPO 局部避障，并逐步接入业务层。

不要表述为“已实现 PPO”“任意场景无碰撞无死锁”或“效率翻倍”；这些结论需要后续实现及对照实验。
