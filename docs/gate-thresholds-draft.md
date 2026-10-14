# Gate 定量门 DRAFT（起草人：agent；状态：待人类拍板）

纪律：本文件所有数字都是**提案**，生效条件只有一个——人类签字。
签字前它们是讨论稿，测试不许 pin，行为不许读。默认状态：预算 0、不放行。

背景：门报告 A/B 是“标准未定义”，C 是“不放行”。本稿把“未定义”变成
可签字项——每个阈值附证据来源＋ rationale＋通过后许可什么。拍板时可以
整项否决（回到未定义），但不能静默改数（改数＝重审 rationale）。

## Gate A：联合扰动（提案）

证据基线（门报告 §A＋附录 F）：估计偏置 30%≈近场均值 0.28 m/s；
max 1.92 未覆盖；0.5006/0.5015 哨兵双侧钉死。

- A1 扰动网格（提议）：威胁速度偏置 {15%, 30%, 60%} × 执行噪声 {0, 0.1, 0.2}。
  rationale：30% 对齐观测均值；60% 覆盖 max/典型速度（1.92/≈3m/s 级遭遇速度，
  取整）；15% 探针阈值附近灵敏度（0.5006 哨兵区）。网格点 9 个，全 assess 级
  可跑（`test_perturbation` 已有 harness）。
- A2 消费侧要求（提议；assess 层**实现不了**，已证）：慢威胁 nominal
  Infeasible（chain −0.73）在 60% 乐观加速偏置下翻成 Feasible（+1.14）——
  `test_perturbation` 4b tripwire 常驻见证。纯评估器“给什么输入算什么数”，
  单调性在 assess 层无定义（garbage in, verdict out）；防御只能活在消费侧，
  二选一（wiring 设计时定）：(i) estimator confidence 门：低 confidence 输入
  的 Feasible  verdict 降级为 Unknown（不看数看戳）；(ii) margin 覆盖偏置误差：
  nominal gap ≥ 偏置诱导误差上界（replay gap 表＋estimation 分布联算）。
  rationale：false-go 的存在证明了“contract → consumer”链条的必要性——
  这不是整洁，是刚需。tripwire 守住洞的形状，关洞归 Gate C。
- A3 通过许可：A2 全绿 → Phase-2 评估器获“扰动下不撒谎”结论，仍不接线。

## Gate B：空间/等待证据（提案）

证据基线（附录 F）：episode 分布 3 关×多种子；sus-fin 2 样本单相位；
pocket 可见 mission 零帧（instrument 窗诊断在案）；latency gap 表；
slip/filter 包络。

- B1 覆盖矩阵（提议形状已冻结，阈值待签）：micro-inf 有（ abundant，不计数）；
  micro-finite ≥5；sustained-inf ≥3 相位；**sustained-finite ≥5 且跨 ≥2 encounter
  family**。当前：sus-fin 2/单相位 → 缺口＝第 2 family＋至少 3 样本。
  rationale：N 在单 family 下不适定（已论证）；2 family 是“模式非偶然”的最小分母。
- B2 pocket 可见性（提议）：≥1 encounter 出现 mission pocket shadow
  feasible>0 帧（含 instrument 窗诊断），且离线 sampler 同几何 feasible
  （存在性＋可见性双确认）。当前：0 → 缺口完整。
- B3 latency 引用（不设阈值）：gap 表（0/1.8/5.4/8.9/17.9cm）作为 wiring 时
  margin 输入；margin 函数本身在 wiring 设计时推导，本门只要求表存在且 pin 住。
- B4 通过许可：B1＋B2 → “等待行为有可验证实例”结论，仍不接线。

## Gate C：行为接线（前置条件，非本稿拍板范围）

C0（提议新增）：parity 运行——wiring 后的行为与 shadow 并行跑 N 个 episode，
分歧（wired 走／shadow 停或反之）逐条记录；放行要求分歧率 < X 且每条分歧
有处置归属。X 未定义（人类定；建议首值 0，即零分歧起步，见下）。
C1–C6：沿用门报告 §C 六项（接口冻结✓／三腿证据✓／A／B／状态机设计／回归），
六项合取，一票否决。

建议的签字顺序：A2（逻辑项，可先签）→ B1 形状（阈值随后）→ B2 → C0 的 X →
C 总门。任何一步不签，维持现状（预算 0、不放行），已有证据全部保留。
