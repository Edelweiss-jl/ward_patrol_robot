# 复杂病房巡逻 Demo

新增代码独立于 `dynamic_ward_demo.py`，不导入或修改原始文件。使用现有虚拟环境中的 PyBullet，不需要额外安装包。

## 运行

在项目目录打开 PowerShell，任选一个运行：

```powershell
.\venv\Scripts\python.exe multi_room_ward_demo.py
.\venv\Scripts\python.exe ring_ward_demo.py
```

也可以在 PyCharm 中使用现有 venv 解释器，直接运行上述入口文件。`complex_ward_demo.py` 是两个新场景共享的建模与导航模块。

## 两个场景

| 项目 | 多病房走廊 | 环形病区 |
| --- | --- | --- |
| 入口 | multi_room_ward_demo.py | ring_ward_demo.py |
| 场地尺寸 | 18 × 14 米 | 22 × 16 米 |
| 病房、床位 | 6 间、12 床 | 8 间、8 床 |
| 空间结构 | 走廊两侧各 3 间双床房，真实门洞和隔墙 | 中央护士站，两侧候诊座椅，上下两排独立房间及环形通行空间 |
| 家具细节 | 床架、床垫、枕头、床头柜、输液架 | 床架、床垫、床旁显示器、设备柜 |
| 动态障碍 | 2 名往返人员 | 3 名往返人员、1 辆推车 |
| 巡检顺序 | N1-1 到 N3-2，再 S1-1 到 S3-2 | ICU1 到 ICU4，再 WARD1 到 WARD4 |

这些是为算法演示设计的抽象布局，不代表真实医院设计规范。

机器人自动穿过门洞，依次前往每张床的巡检点，停留约 0.6 秒后记录 `bed_arrival` 事件。绿色圆点为待巡查床位，蓝色表示已完成。GUI 中绿色折线是本次规划路径，红色圆柱为机器人，黄色标记表示朝向。完成后窗口保留，按 ESC 退出。

## 交互按键

先点击 PyBullet 窗口获得键盘焦点。

| 按键 | 功能 |
| --- | --- |
| O | 在未来路线的安全空位放置临时障碍；找不到合适位置时在终端提示 |
| C | 清除临时障碍并重规划 |
| R | 手动重规划 |
| 空格 | 暂停/恢复仿真，包括人员移动 |
| V | 恢复默认俯视相机 |
| ESC | 退出 |

建议演示：启动后观察穿门巡查，按 O 观察绕行或等待，再按 C 观察恢复。临时障碍封住唯一入口时等待是预期行为。重开程序可重新演示完整巡查。

## 导航与扩展边界

- 使用 0.25 米栅格、八邻域 A*、机器人半径及安全距离膨胀，禁止斜穿墙角。
- 路径受阻后重规划，无可行路径时停止并定期重试；每次移动前检查整段运动是否安全。
- 人员与推车沿预设路线运动，也会礼让机器人；受阻等待 1 秒后折返，避免相互卡在门口。这是规则行为，不是学习得到的策略。
- 使用运动学位置更新，没有轮式动力学、真实传感器、SLAM 或临床监测。
- 当前未训练或接入 PPO，未连接 MQTT、数据库或 Web。到达事件暂存于 `Simulation.events`，可通过 `--report` 导出 JSON，便于后续对接。
- 碰撞导航使用保守的家具占地矩形。枕头、监护显示器等装饰不额外参与碰撞。

## 无窗口验证和预览

```powershell
.\venv\Scripts\python.exe multi_room_ward_demo.py --headless --seconds 240 --screenshot demo_outputs/multi_room.png --report demo_outputs/multi_room_report.json
.\venv\Scripts\python.exe ring_ward_demo.py --headless --seconds 240 --screenshot demo_outputs/ring_ward.png --report demo_outputs/ring_ward_report.json
.\venv\Scripts\python.exe -m unittest test_complex_ward_demo -v
```

`--seconds` 是无窗口模式的最大仿真时间，完成巡查后提前结束；超时未完成会返回退出码 1。截图是初始场景的离屏渲染，PyBullet 调试标签及路径线不会出现在离屏截图中，但会在 GUI 中显示。JSON 中的秒数是仿真时间，不是程序运行耗时。

测试覆盖两种布局的逐床连通性、人员路线与家具分离、临时障碍、巡检点完全受阻、障碍清除后的整轮动态巡查，以及机器人与家具和人员的碰撞检查。
