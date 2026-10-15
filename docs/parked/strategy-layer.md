# 策略层分叉提案（待批，未接线）

状态：ACCEPTED-A（2026-10 决策：Phase-1 只观测继续；Phase-2 接线仍冻结，等三门＋点名）。
tracker 去假 trio（slot＋birth、bbox extent、CTRV）的重启触发器见
`docs/parked/tracker-defake.md` §5——本提案获批即满足其“策略层立项”半条；
另半条（可验证需求＋三门同过）在 Phase-1 影子数据出来后才谈。

## 1. 问题（实锤，非推测）

两旗舰遭遇死因已定案：正确感知＋竞速误判＋无停止线（park 报告 §2）。
放行看当前距离，不看通过窗口内的威胁相位——慢车要更宽的窗口，
逻辑给的却是同一标准。调参修不好，归 planner-margin／执行模型账。

## 2. 形状（行业同形，三层切分）

* Mission：全局路线，只管 where（已有 A*，不动）。
* Behavior（新增，robot 侧独立模块）：只输出约束
  HOLD_AT / PROCEED / 限速窗，不管 how。
* Local（DWA）：消费约束做轨迹优化，约束语义是虚拟墙，不是打分微调。

策略三件套：停止线＋进入时机（gap-acceptance，看整个通过窗口相位）
＋等待点（必须在 swept 区外——pocket 教训：停在 swept 区里内容动不了）。

## 3. 两阶段门（冻结线）

* Phase-1：影子只观测。assess 输出 verdict 词表 Observed*，永不
  Feasible/Allowed；mission/DWA 零消费，树保持全绿。
* Phase-2：消费接线。HOLD_AT 以虚拟墙进 DWA；验证三门：
  旗舰双关 pinned 绿＋30 种子＋速度 RMSE vs 真值，三者同过才算。
* 非目标：reactive actors（chase/flee/RVO）各需自家门，本提案不含；
  不调物理/阈值/内容去凑数字。

## 4. 分叉选项

* A：按 §2–§3 立项，先 Phase-1 影子（只加观测代码＋单测＋门报告）。
* B：继续 park，策略需求等真车/真日志到了再谈；仿真侧只保留 record-only。

## 5. 停止线放置实测（test_stop_line_study，observe-only）

* signal_cross/recommended：14 等全在廊道外（干净）。
* signal_hold/watch/recommended：各 490 等，其中 25 步（5%）停在
  swept 廊道内（威胁路径 ±0.9m）。pocket 判断在实数上成立，但只是小分量——
  主矛盾仍是进入时机（窗口相位），停止线是次矛盾。
* what-if 回放（test_stop_whatif，纯重算零接线）：hold 全程 2626 评估中
  实际 Occupied 1313 步；同速退到停止线候选点后仅 236 步转 Clear（18%）。
  诚实解读：位置后退只解决小部分，停止线的真正价值是停住等相位
  （v=0 不进窗口）＋在 Clear 上出发——主矛盾确认是进入时机，停止线次之。
* 放置规则（提案，未施工）：冲突区入口前 (v^2/2a＋1.0m)；
  grip 胎 2.0m/s 巡航＝0.48＋1.0＝1.48m。候选点必须验在廊道外。

决策记录：A。B 废弃。
## 6. Phase-2 接线实录（已施工＋验证，用户授权）
* 接线：governor（一侧刹车，只发 HOLD_AT）＋DWA 虚拟墙（v<=sqrt(2ad) 钳速）。
  social 行人不治理；停稳按 0.5m/s 出发地板评估；停止点按预测占用全帧算
  （首版按当前位置停车曾在 hold 上把车停进威胁怀里撞了——已修，教训入档）。
* 全表 --seeds 3：cross 净空 0.61→0.93、big 0.20→0.54、watch 默认 1/3→2/3、
  hot 车复活；hold 默认 2/3→1/3 经 30 种子核查为相位彩票（去 governor 17/30，
  接线 14/30，本来就 ~50%，3 种子 2/3 是运气）。
* 外部评审对照（2026-10，三条全过）：抖动有 0.5s 迟滞（kReleaseClears，
  test_entry_governor 钉 25 帧才放）；DWA 采样窗即被钳（v_max 就是 clamp 后的
  speed_cap，采样不出超限轨迹，不存在事后刹不住）；措辞采纳评审版——governor
  只输出 HOLD_AT＋停止点语义约束，sqrt 钳速是消费侧（mission）的解释，不在行为层。
* 已知局限：威胁长期 stale 时 governor 静默释放后不再二度咬住（seed 1 实录：
  41s 咬住一次，71s 等到 73.3s 撞）——根子在感知保鲜，不在刹车逻辑，
  这正是 tracker trio 仍 park 的理由，不单开。
