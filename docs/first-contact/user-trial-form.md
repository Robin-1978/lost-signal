# First Contact 目标用户试用记录

每位参与者使用同一 Release 包、同一 `first_contact` 配置和同一语言版本。记录编号不填写姓名；结果用于判断理解闭环，不用于排行榜。

## 试用协议

1. 试用前回答三道知识题，不阅读开发者文档。
2. 从冷启动完成 First Contact：跳过英雄镜头、做一次预测、运行 baseline 和 variant、查看复盘。
3. 只在用户主动询问后解释 ROBOT/SIM authority；记录用户第一次自发说出的因果解释。
4. 试用后回答同一组三道知识题，并填写体验反馈。

## 前后知识题

主持人只读题目，不补充术语。每题 1 分；回答包含“测量/表面距离/时间戳”
等关键概念即可得分，提到物体身份或仿真真值由 LiDAR 直接提供则该题得 0 分。

| 题号 | 题目 | 得分要点 |
|---|---|---|
| Q1 | 激光雷达给机器人什么信息？ | 带采集时间的表面距离测量；不包含物体身份或仿真世界真值。 |
| Q2 | “机器人到达”和“仿真验收”为什么要分开？ | ROBOT 只能依据自己的观测/估计宣布完成；SIM 另行检查物理停车、碰撞和净空。 |
| Q3 | 为什么要比较 20 ms 与 100 ms 两次运行？ | 世界、机器人、初态和种子固定，只改变扫描周期；用两次实测的安全性、耗时和净空判断差异。 |

记录每题原话或简短摘要，再填写总分。前测不得解释答案；后测允许用户引用复盘中的证据，但不替用户补全因果链。

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
