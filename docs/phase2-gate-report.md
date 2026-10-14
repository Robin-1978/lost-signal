# Phase-2 Gate Report（门禁审计制，living document）

> 格式冻结：每道门独立结论，不用 CI 全绿替代安全验收；最弱门原则。
> 数字判据未定义处明确标“未定义”，不凑数。

## Gate 0. 经验接地 — 结论：缺失（前置于一切）

本报告 A/B/C 的全部证据都是 sim 内一致性（模型对模型、代码对代码），
不能证明关于现实的任何陈述。排序其认识论地位：
- 证伪有效：FF 阴性 verdict、trigger-count 盲区、aliasing＋Qvv bug、
  WaitUnsafe 退化——sim 里抓到的 bug 就是 bug，不需要真机确认。
- 契约验证有效：模块间组合正确性（傻子才用真车调接口）。
- 差距量化在 sim 内为真：0.3s bias 是两条代码路径的差，不是物理claim。
- 以下无效：摩擦/噪声/时序数值外推、真实风险阈值、安全声明。
A/B/C 是 sim-consistency 门；经验接地（真车／真 log 回放／器件手册噪声）
是 0 号门，位于一切之前——它不开，A/B/C 的通过在物理上无意义。
log-and-compare 格式（见 sim2real.md）是现成的桥：它的价值不是证明对，
而是准备好被现实证伪。游戏侧（21 关好玩／教学）不归本报告：它的经验证据
是 playtest（真人），不是物理。两条轨道，两种证据，不混。

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

## B. Shadow—空间观察 — 结论：样本不足（观察仪器已上线，3 关×3 种子的 departure 分布；pocket verdict 分布仍单关）

- 空间 pocket 采样已作为只读观察挂载（仅等待周期，`--seeds 3` 隔离实验证明
  行为零漂移：与挂载前 6 个 run 逐字一致）。
- 终关首批分布（184 等待帧，replay 可复算）：verdict refused 138 / incomplete 46 /
  feasible 0；三腿原因 WaitUnsafe 1336 / HorizonShort 904 / Clear 354 /
  BlockedInWindow 106（GoNow/Wait 在腿层级不用；Unassessed/StaticBlocked/Invalid 均为 0）。
  解读：擦身几何下附近口袋同样被扫过或链条超窗——与 Phase-1 结论一致，
  不是仪器故障（truncation 已用 stride＋上限排除为系统性原因，总数 4/6 未触顶）。
- 放行误差首测（`test_release_error`，2 关×3 种子，episode 自带入口速度/距离）：
  85 episode 中 76 个初值 inf（微等待起点不可评估——与 WaitUnsafe 主导一致）；
  9 个有限初值误差全为负（实际 0.1–0.15s vs 预测 2–3s：DWA 蠕行释放远早于
  shadow 预测）。口径声明：该误差混杂预测误差与策略差异（DWA 本来就不按
  shadow 等），是“shadow-vs-实际”偏差，不是纯预测误差；已按入口速度 bin
  记录，不做标定。
- 多关积累（test_shadow_accumulation，mission 未碰）：signal_hold × 3 种子
  45 episode，格 [35,5][4,1]——sustained-finite 格首样本（前两关为空集）。
  seed 分解（42/43/44）：[12,2][2,1]／[11,3][0,0]／[12,0][2,0]——sustained-finite
  仅存于 seed 42（策展相位），sustained 格 2/3 种子出现。coverage 读数：micro 两格满、
  sus-inf △、sus-fin ✗（单样本单 seed）。结论维持样本不足，且明确了缺口形状。
  与 release_error（lost_signal/signal_echo × 3 种子）合计 3 关×3 种子。
  return_route（RO-2）探针零等待，无分布可积累，未纳入（不断言脆弱零）。
- 仍缺：DRAFT 量化拍板。预算 0 不变，不放行不变。

## C. 行为放行 — 结论：不放行

1. 搜索＋三腿接口冻结 —— ✓（`LocalCandidate`/`check_candidate`/三腿语义）。
2. 三腿证据完整、缺失不升级 —— ✓（spin 完整；Ackermann 曲率执行评估已落地：A*＋Rmin 圆角＋伺服滞后自行车跟随，扫 SIMULATED 轨迹，kCapsViolated／Unknown 语义冻结；RO-2 pocket shadow 起步滞后 0.34/0.39m ObservedDeviated 在案）。
3. 联合扰动通过＋裕量依据 —— ✗（A 未定义）。
4. Shadow—空间观察达量化门槛 —— ✗（B 未定义）。
5. 行为状态机（中断/重估/失败） —— ✗（未设计，gates 全过才写）。
6. pinned 回归＋mission 改动授权 —— ✓ 回归（29/29 CTest、20/20 pytest、79.84/0.620 逐字），mission 零改动（本周期全部纯函数＋测试）。
六项合取，一票否决 → 不放行。

## D. 工程完整性

- 范围：纯函数＋测试＋plan/doc；mission 零改动；diff 净增，零删除生产代码。
- build / test / report：29/29 CTest、20/20 pytest、pinned 表逐字（79.84/0.620）。
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
| 本周期（放行误差＋入口分层，episode 修漏报） | 1 | 0 | 0 | 0 |
| 本周期（打滑 shadow：履带 upper-bound 阴性） | 1 | 0 | 0 | 0 |
| 本周期（Ackermann transit 曲率执行＋RO-2 shadow） | 1 | 0 | 0 | 0 |
| 本周期（estimator contract＋replay/latency＋filter＋chain） | 1 | 0 | 0 | 0 |
| 本周期（slip residual＋生产流＋激励矩阵＋shadow 多关积累） | 1 | 0 | 0 | 0 |

中继 0 的含义：本片在冻结约定内（只读观察＋隔离实验证明），无架构分叉、
无门禁结论变化、无 stop-loss——按约定不打扰评审。

## F0. Family C 否决记（signal_pocket，回退完成）
- 长扫 encounter 下 mission pocket shadow 525 帧 feasible 0，主因 HorizonShort
  （instrument 窗覆盖不了 transit＋hold＋rejoin 链）；离线存在性（20s 窗、
  chain 0.62–1.68）与 mission 零可见并存——差的是仪器窗，不是几何。
  加窗追可见性＝调参观察预算，已禁止。关卡另因对比弱＋文案承诺落空否决，
  与仪器结论相互独立。sus-fin 仍单相位 2 样本。

## F. 不确定性证据表 v1（evidence table，不是 budget）

命名纪律：本表无消费者、无 N 阈值，任何 `margin += f(...)` 须在 wiring 时
推导＋pin——此前它只能叫证据表，更名 budget 的那天就是 Phase-2 接线的那天。

### F1. 等待 episode 分布（departure shadow，关×种子）

| 关 | 种子 | micro-inf | micro-fin | sus-inf | sus-fin |
|---|---|---|---|---|---|
| lost_signal＋signal_echo（release_error） | 2 关×3 | 72 | 9 | 4 | 0 |
| signal_hold（accumulation） | 42–47 | 70 | 10 | 8 | 1（仅 seed 42） |
| signal_cross（Family B，30 种子扫描） | 11–40 | 见正文 | 见正文 | sustained 13/21 | 0（WaitUnsafe 主导，停车点在 swept 区内） |
| signal_watch（accumulation） | 同走廊×3 | 35 | 5 | 4 | 1（同相位，相关非独立） |

signal_cross 30 种子（default/rec 各 30）：成功 30/30、碰撞 0（否决门过），
default 格 [24,3][13,0]、recommended [60,19][21,0]，maxwait 4.5/4.7s；
first-reason WaitUnsafe 66/100（approach 停车点被横扫）、Blocked 15、GoNow 17。
新 encounter 家族（交叉等待）成立，sus-fin 仍空——内容动不了 DWA 停车点。
覆盖读数：micro 两格满；sus-inf 在 6/9 种子相位出现；sus-fin＝2 样本、
同走廊同相位——单相位依赖，Gate B 缺口形状即此。

### F2. 时间误差（replay，同一轨迹×延迟）

latency 0/20/50/100/200ms → gap 误差 0/1.8/5.4/8.9/17.9cm（单调为正：
planner 高估 margin，乐观危险方向）。dropout 末端窗 0/50/150/300ms：
预算内 Fresh 带 0.06m 残差；超预算 hold 误差≈dropout×速度、CV≈0、flag 拒估。

### F3. 执行偏差（RO-2 shadow，assess vs 生产 rollout）

直道 max 0.34m／90°弯 0.39m，ObservedDeviated（瞬时巡航 vs 静止 spool-up，
时长按构造相等，滞后体现在空间）。归属：planner-margin／执行模型账。

### F4. 估计与打滑（filter＋observer，生产流）

filter 合成真值 RMSE 0.029→0.007、3σ 一致 291/300；slip 残差包络：
铺装/沙地稳态 0.0024/0.0064、搓擦 yaw 0.175、蠕行全拒估。confidence 载体
（1/(1+trace)）为报告口径，未标定覆盖率——标定待 filter 真流。
