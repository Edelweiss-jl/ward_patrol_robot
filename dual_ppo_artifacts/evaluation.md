# 第三阶段实测对照

模型 SHA256：`0111e68c538be8f9c01b8db0ac58a2454182544a5defd2cb4314d75f34ff049a`

| 场景 | 人员 | 控制器 | 成功/回合 | 成功平均秒 | 安全拦截帧 | 机器人重叠帧 |
| --- | --- | --- | --- | --- | --- | --- |
| corridor | False | ppo | 5/5 | 21.06 | 1 | 0 |
| corridor | False | astar | 5/5 | 21.06 | 0 | 0 |
| corridor | True | ppo | 5/5 | 28.45 | 0 | 0 |
| corridor | True | astar | 5/5 | 28.45 | 0 | 0 |
| ring | False | ppo | 5/5 | 44.92 | 64 | 0 |
| ring | False | astar | 5/5 | 44.92 | 0 | 0 |
| ring | True | ppo | 1/5 | 21.98 | 49 | 0 |
| ring | True | astar | 1/5 | 21.98 | 0 | 0 |

安全拦截为动作被安全闸停车的仿真帧数，不是物理碰撞次数。
环形为跨布局测试；训练只覆盖配置中记录的布局。
