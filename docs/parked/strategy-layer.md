# 策略层分叉提案（待批，未接线）

状态：PROPOSAL。Phase-1/2 接线冻结，在用户点名前不施工。
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

请点名 A 或 B。不点名＝冻结维持，本文件保持 PROPOSAL。
