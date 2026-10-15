# Tracker 去假 Park 报告（证据保留， trunk 已回退到绿）

状态：PARKED。主干 34/34 CTest + 20/20 pytest + 22 行旧数逐比特一致。
回退原因不是感知失败，而是两个旗舰遭遇暴露**规划乐观**（见 §5），属策略层事项，
按纪律不 solo 接线（Phase-1/2 门），先 park，证据全留。

## 1. 去掉的作弊（全部定位到文件行，复用时查）

- 关联作弊：`src/hw/sensors/sensor.cpp` 射线求交直接写 `dynamic_hit`（哪条 actor）。
- 半径作弊：`mission_simulation.cpp` 确认/观测用 actor 真半径。
- 有限差分速度：`dynamic_obstacle_map.cpp` 单拍差分＋0.55 blend＋反转 snap，
  5Hz 下 ±1.5 m/s 垃圾（真值 0.55）。

## 2. 已验证有效的部件（复用清单，按置信排序）

- 自研 NN 关联（1.0m 门，互斥，纯函数）：5 关×双配置×30 种子约 43 万射线，
  一致率 97.3–99.9%，分歧全是 track 生死时刻。测试 `test_track_association`
  已随回退删除，实现见 git 历史（association parity commit）。
- 聚类＋0.9m 合并＋静态背景剔除（真值静态图紧门限）：单 cluster 率 100%
 （lost 全程单 track-0），解决了 grazing 角碎裂和墙体 ghost。
- per-track CV Kalman（Q=1.0 内容接地，R=传感器项＋半径²/ray 采样项）：
  速度从 ±1.5 收敛到真值附近（+0.5 vs 真值 +0.55，见探针 veldense）。
- 方向反射界（只在见过掉头的一侧反射）：修了东端 phantom retreat 误报。
- 内容派生新生先验（本关 actor 最大半径，ODD 知识非 per-ray 真值）。

## 3. 关键测量（探针名，/tmp 失效后按名重写）

- `ls_track`：位置误差 0.04–0.23m（好）。
- `ls_ghost`：单 slot 全程，零多 track 帧（无 ghost）。
- `veldense`（KF+R 诚实后）：速度稳定真值附近。
- `commit_win`：撞前 2 秒 robot 静止等待，track 完美——死因不在感知。
- `velprof`：60 秒附近 stop-go 抖动（ latch 时代，已 revert）。
- 旧管线 120 种子 big_freight 也是 119/120（-0.00 擦碰类）：完美主义无基线。

## 4. 失败机制（ perceived 完美仍死 ）

走廊/路口遭遇是到达竞速：DWA 用正确预测算出“能过”然后以 3cm 落败。
旧管线靠噪声误报早停（南侧安全位）幸存；新管线正确评估后晚停（杀戮区内）。
修感知只会换噪声实现，不改竞速本质——这是规划乐观＋无停止线，
即 Phase-1/2 策略层事项（见 §5），不是 tracker事项。

## 5. 重启条件（触发器，非时间表）

策略层立项（stop line＋entry-timing 消费＋wait-spot，见 dynamics-plan 策略层节）
并给 tracker 提可验证需求时，按本报告 §2 清单复用部件、§3 探针复测，
门禁：旗舰双关 pinned 绿＋30 种子＋估计质量（速度 RMSE vs 真值）三者同过。
在此之前：simulated dynamic_hit 等作弊保持原样，门禁继续看门。
