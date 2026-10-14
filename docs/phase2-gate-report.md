# Phase-2 Gate Report（门禁审计制，living document）

> 格式冻结：每道门独立结论，不用 CI 全绿替代安全验收；最弱门原则。
> 数字判据未定义处明确标“未定义”，不凑数。

## A. 联合扰动 — 结论：标准未定义（测试矩阵通过，标定依据缺失）

测试矩阵（`test_perturbation` assess 级 ＋ `test_departure_replay` 真值闭环）：

| 场景 | 扰动 | 结果 |
|---|---|---|
| 名义 pocket 链 | 无 | Feasible，chain 0.6696（rejoin 绑定），end 9.95 ≤ 12 |
| 估计偏置（威胁慢 30%） | 预测输入 | Rejoin 首败 BlockedInWindow，chain −0.73；transit/hold 不受影响 |
| 执行噪声 0.15（transit） | 自车时序轨迹 | 1.27 → 1.13，仍 feasible，rejoin 不受影响 |
| 联合（偏置＋噪声） | 双输入 | 仍由 rejoin 首败；transit 带噪仍 feasible——扰动各打各的对象 |
| 阈值边界（同 pocket，5% 偏置） | 预测输入 | 0.5015 → 0.3754，不可放行（名义过线零鲁棒） |
| 时间窗边（4m 路径） | H=3.9/4.0 | HorizonShort / GoNow，单帧翻转 |
| pocket 真值联合（估计快 40%） | 真值重算 | 出发 0.2，真实间隙 0.5006——边界哨兵，双侧钉死 |
| pocket 保守偏置（估计慢 20%） | 真值重算 | 安全但晚到（time miss），间隙 0.92 |
| 名义＋0.1 盲跟踪摆动 | 真值重算 | 间隙 0.52 → 0.56，存活 |
| 等待点失效（符号反转估计） | 真值重算 | “等 0.6 再走”撞上穿等待点的威胁，真值碰撞 −0.78 |

扰动幅度及标定依据：**多关多种子证据已落地，未成标准**（`test_estimation_error`，
lost_signal＋signal_echo × 3 种子＝6 run，共 3747 样本，成功失败皆收录）：
近场中速（n=2945）速度误差均值 0.28、最大 1.92 m/s；
远场中速（n=802）均值 0.59、最大 1.96 m；慢速/快速 bin 零样本——
当前内容里没有慢/快动态威胁，DRAFT 分层暂时填不满，这是内容覆盖缺口，
不是标定结论。当前 30% 偏置 ≈ 近场均值；max 哨兵（0.43 miss）继续有效。
速度分层、样本量、漏检率、误差上界：**仍未定义**。本门维持“标准未定义”。

## B. Shadow—空间观察 — 结论：样本不足（观察仪器已上线，单关数据）

- 空间 pocket 采样已作为只读观察挂载（仅等待周期，`--seeds 3` 隔离实验证明
  行为零漂移：与挂载前 6 个 run 逐字一致）。
- 终关首批分布（184 等待帧，replay 可复算）：verdict refused 138 / incomplete 46 /
  feasible 0；三腿原因 WaitUnsafe 1336 / HorizonShort 904 / Clear 354 /
  BlockedInWindow 106（GoNow/Wait 在腿层级不用；Unassessed/StaticBlocked/Invalid 均为 0）。
  解读：擦身几何下附近口袋同样被扫过或链条超窗——与 Phase-1 结论一致，
  不是仪器故障（truncation 已用 stride＋上限排除为系统性原因，总数 4/6 未触顶）。
- 仍缺：多关积累、速度分层、DRAFT 量化拍板。预算 0 不变，不放行不变。

## C. 行为放行 — 结论：不放行

1. 搜索＋三腿接口冻结 —— ✓（`LocalCandidate`/`check_candidate`/三腿语义）。
2. 三腿证据完整、缺失不升级 —— △（spin 完整；Ackermann 曲率模型缺失→Unknown，诚实但未闭环）。
3. 联合扰动通过＋裕量依据 —— ✗（A 未定义）。
4. Shadow—空间观察达量化门槛 —— ✗（B 未定义）。
5. 行为状态机（中断/重估/失败） —— ✗（未设计，gates 全过才写）。
6. pinned 回归＋mission 改动授权 —— ✓ 回归（15/15、20/20、79.84），mission 零改动。
六项合取，一票否决 → 不放行。

## D. 工程完整性

- 范围：纯函数＋测试＋plan/doc；mission 零改动；diff 净增，零删除生产代码。
- build / test / report：15/15 CTest、20/20 pytest、pinned 表逐字（79.84/0.620）。
- Stop-loss：未触发（一票：任一 pinned 翻转即 revert，无需评审）。
- 新风险→归属：Ackermann 曲率执行模型（架构分叉，待评审讨论后开工）；
  一般性边界探索进 backlog，不自动成为出口门槛。
- 本周期 mission 变更：空间 pocket 只读观察（快照 3 整数＋9 原因 bins）；
  隔离证据 `--seeds 3` 与挂载前逐字一致。

## E. 审计账本（审计制生效起）

| 周期 | 机械片落地 | 中继评审 | 回退 | 逃逸 |
|---|---|---|---|---|
| 审计前（估计，用户侧可见） | ~10 | ~10（逐片） | 0（最后 revert 为 scored-waiting stop-loss） | 0 |
| 本周期（空间 shadow 挂载） | 1 | 0 | 0 | 0 |
| 本周期（估计误差证据＋max 哨兵） | 1 | 0 | 0 | 0 |
| 本周期（误差多关多种子＋速度分层） | 1 | 0 | 0 | 0 |

中继 0 的含义：本片在冻结约定内（只读观察＋隔离实验证明），无架构分叉、
无门禁结论变化、无 stop-loss——按约定不打扰评审。
