# First Contact 目标用户试用记录

每位参与者使用同一 Release 包、同一 `first_contact` 配置和同一语言版本。记录编号不填写姓名；结果用于判断理解闭环，不用于排行榜。

## 试用协议

1. 试用前回答三道知识题，不阅读开发者文档。
2. 从冷启动完成 First Contact：跳过英雄镜头、做一次预测、运行 baseline 和 variant、查看复盘。
3. 只在用户主动询问后解释 ROBOT/SIM authority；记录用户第一次自发说出的因果解释。
4. 试用后回答同一组三道知识题，并填写体验反馈。

## 记录表

| 字段 | 值 |
|---|---|
| `trial_id` | 例如 `fc-001` |
| `package_sha256` | 交付包 SHA-256 |
| `locale` | `zh-CN` / `en` |
| `start_utc` / `end_utc` | ISO-8601 |
| `prediction_choice` | `safe_tradeoff` / `equivalent` / `unsafe_variant` |
| `prediction_confident` | `yes` / `no` |
| `variant_completed` | `yes` / `no` |
| `debrief_completed` | `yes` / `no` |
| `causal_explanation_unprompted` | 原话或 `none` |
| `pre_knowledge_score` / `post_knowledge_score` | 0–3 |
| `blocking_issue` | 简短描述或 `none` |

最小验收汇总使用完成时长中位数、预测选择分布、variant 完成率、复盘完成率、知识题前后分数和阻塞问题原文。不要用星级、游玩时长或主观喜好替代这些指标；少于 10 名参与者时只报告观察结果，不宣称教学效果已验证。
