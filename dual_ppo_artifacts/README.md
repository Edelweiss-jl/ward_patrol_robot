# 第三阶段交付产物

- `ppo_dual.zip`：由 `train_dual_ppo.py` 真实训练的集中式联合 PPO，52维输入、两台各9动作。
- `training_config.json`：准确训练配置、依赖版本、实际步数、墙钟用时、完成标志。
- `monitor.csv` / `progress.csv`：训练回合与优化过程原始日志。
- `evaluation.json` / `.csv` / `.md`：独立种子下PPO和A*的真实对照，JSON包含模型SHA256和动作统计。
- `results.png`：根据上述日志生成的汇报图。
- `scene.png`：本阶段复用的静态复杂走廊初始场景，红蓝各一台机器人。
- `validation.md`：本次自动验证记录及结果解释。

完整操作、算法和局限见上一级 `README_dual_ppo.md`。不要使用第二阶段单机器人入口加载本模型。
