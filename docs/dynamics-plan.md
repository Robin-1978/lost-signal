# 总体计划：仿真真实化、模块化与内容规模化

> 原则：内容归 JSON、契约归代码；硬件槽位决定“装什么”，接口决定“怎么驱动”；
> 算法接口只在第二个实现出现时才开；官方配置结论不变，数值漂移后重标定。
> 方法纪律：混沌遭遇下重构前后必须全表比对（30 组），单种子断言外加多种子证据；
> 文本等价≠行为等价，差分测试先行。

## 0. 原则（三次讨论定稿，不再改）

1. **不硬编码**：按车型/硬件该变的数据进 JSON；纯方法常量（采样分辨率、迟滞阈值、
   看门狗秒数）留代码，注释写明推导 + 公式进 `docs/design.md`。
2. **PID 是算出来的**：增益 = 装配物理的纯函数，DIY 新硬件无需手调；玩家永远看不到 Kp。
3. **玩家只调风格**：驾驶风格无极条 s∈[0,1]，三锚点分段插值；官方配置落锚点，行为不变。
4. **失败必须诚实**：观测、跟踪、预测、避让全链无真值泄漏；诊断数字来自实测事件。

## 1. 接口地图（统一接入接口）

### 硬件侧：槽位是唯一的接入点
* 组件 = `id/title/description/cost/behavior（数字表）/visual`，`BuildSelection.parts` 是
  `slot→id` 映射，任务/校验/UI/存档全部按槽位名泛型处理——**加新槽位只需改 JSON
  + 4 处 schema 枚举**（components、robot slots、level required_slots、presentation columns），
  C++ 零分支（`selected_id`/`assign_id`/校验循环/存读已泛型化）。
* 槽位注册表见各 schema；新增槽位检查清单：robot slots、columns、预算、解锁授出、
  回归计数、`test_content_catalog` 断言。

### 玩家扩展边界（结论：三条线，一条已通两条半通）
* **加“世界”自由，加“形状”要配合**：新地图/关卡/地面矩形/装饰实例只管丢 JSON
  （`map` 字段指过去，validator 全链检查）；新几何图元（圆柱/多边形/高度场）
  要 C++（渲染＋碰撞＋schema 三处）。
* **加“数据”自由，加“语义”要配合**：同槽新件、新槽（4 处 schema 枚举）、新整车
  纯 JSON，零 C++ 分支；新底盘类型、新行为语义（数字有人读才生效）、
  新目标/评分类、新图元走 §1 清单，在稳定接口后实现。
* 判定法则（一句话）：**只加数据不加语义 → 不用 C++；加语义 → C++ 在稳定接口后实现。**
* 缺的不是能力是工具：手写 JSON 撑 12 关行，撑 20 关和社区供稿不行（见 §4 编辑器项）。

### 执行器/传感器接口（已实施）
* `ChassisModel`（`src/hw/chassis/`）：`step`/`telemetry`/`caps`——差速（双轮 PWM）
  、阿克曼（驱动 PWM＋舵机角度）、履带（双履带＋搓擦阻尼）在同一接口后；
  未知 `drive.type` 直接报错。
* `LidarBackend` + `SimLidar`（`src/hw/sensors/`）：与直调逐比特一致，已验证；
  编码器/IMU 后端是 Phase A2（排后）。
* `ChassisCaps{can_spin}`：跟随恢复逻辑按能力分支（能转打转、不能转继续倒车），
  单测覆盖两种。

### 算法接口（只在第二个实现出现时开）
* `Predictor`（规划中）：等速＋反射是默认实现；协同转弯/意图预测是已知下一实现。
  上次独立因文本等价误判翻车，已回退——**重做时先写差分测试**（新旧预测帧在
  固定观测序列上逐字节比对，含非零 age、多目标、反射边界），再搬代码。
* Planner（A*）：单实现，不开接口；RRT/JPS 出现时再开。
* PID/DWA 内部：单实现方法代码，不开接口。

### 加新底盘检查清单（已走三遍：差速/阿克曼/履带）
1. `src/hw/chassis/` 新实现 `ChassisModel`（`step`/`telemetry`/`caps`）＋单测
   （直行/打转能力/打滑 flag/双跑确定性）。遥测能套双驱动布局就套，
   套不上才加分支——分支按 `drive_type` 显式写，不按能力猜。
2. 参数进机器人 JSON（硬件数）＋`robot.schema.json`＋loader＋validator；
   回路增益必须从物理推导，禁止手调 Kp。
3. `mission_simulation.cpp` 工厂分支＋yaw 上限（DWA/手动/钳制统一走它）＋
   航向 PID 输出单位（角速度/转角跟底盘走，见 `follow.hpp` 约定）。
4. 内容车＋关卡（`presentation.robot` 指过去）＋官方双配置＋全表/多种子平衡。
5. 认知层：非原地打转画转弯圆；控制卡新遥测形状要分支（右栏不许编 fiction）。
6. 文档三件套：design 公式、validation 基线、plan 台账。
* 能否 plugin（.so 动态加载）？今天不能，也不需要：底盘是确定性内核，
  同一二进制跑遍所有主机会省掉 ABI/信任/复现三套麻烦；DIY 玩家通过 JSON
 （轮胎/电机/材质/关卡）扩展，不碰底盘。等社区出现第四个底盘作者再开
  registry 接口——接口只在第二个“外部”实现出现时开，本条规则对人也适用。

## 2. 模块目录（已实施）

```
src/hw/chassis/   电机/差速/阿克曼/履带执行器 + 车辆动力学（力/摩擦/惯性）
src/hw/sensors/   雷达/编码器/IMU 传感器模型 + HAL 接口
src/algo/         A*/DWA/PID/跟随/栅格/动态跟踪（纯算法，无硬件概念）
src/game/         内容加载/任务仿真/诊断回放（不动） 
src/client/       3D 探索层/2D 认知层/TGUI 玩家界面（不动）
```

规则：`hw` 不依赖 `algo` 之外的层；`algo` 不碰硬件概念；加新底盘只动 `hw/chassis`，
加新算法只动 `algo`，换 HAL 后端只动 `hw`。

## 3. 已完成台账（验证状态）

* 阶段一去硬编码：航向/轮速 PID→自整定、能耗/到达判定/地面标签/跟踪记忆→JSON。
* 阶段二动力学：电机扭矩曲线、轮胎×地面综合摩擦、质量/转动惯量；DWA 半径与
  横摆加速度来自机器人 JSON（0.38/8.0），旧 `0.3` 常量已删。
* 无极风格条 + 驾驶风格 UI（Slider）+ 事故/复盘四阶段。
* DIY 车库：奖励→库存→栏位过滤、装配跨关持久、卡片直选、3D 预览同步装配。
* DIY 车库：奖励→库存→栏位过滤（ validator 跨关拥有检查）、装配跨关持久
 （`last_builds`）、卡片直选、3D 预览同步装配；L4/L5 预算随槽位成本调天花板。
* 模型库：轮胎 5 种、电机 3 种（标准/扭矩/高效）、算力 3 种、雷达 4 种、IMU 2 种。
* IMU：陀螺独立随机流、按滑移加权互补融合；加速度计 specs 已进内容
  （basic 0.15 / precise 0.05 m/s²），`SlipDetector` 算法已单测，
  进游戏接线等四分之一车的胎面状态（见 §4 第 2 项）。
* 章节：第一章认识部件（5 关零压力）＋第二章真上路（10 关），P0 切片为毕业考；
  `chapters.json` + 关卡 `chapter` 字段驱动分组。
* 阶段四管线：`generate_levels.py`（模板+参数表+`--check`）与 `level_report.py`
  （全关平衡表）；首批信号回响已接入。
* 工具：`format.sh` + `.clang-format`（1 空格），全树已统一。
* 预测器独立：`src/algo/predictor.*` 纯函数＋五场景差分测试常驻；跟踪器只出快照。
* 多种子线束：`--seed` 覆盖＋`level_report.py --seeds K`＋确定性自检＋C++ 种子断言；
  终关 5 种子证据（推荐 3/5 成功）是结论门。
* 地图存盘路：`serialize_map_canonical`＋`test_map_roundtrip` 逐字节一致；
  `MapEditor`（纯逻辑＋单测）＋客户端编辑模式 v1/v2（移动/增删/缩放/存盘 reload 校验）。
* sim-to-real 地基：`docs/sim2real.md`（缺口/标定/里程碑）＋`tools/sensitivity.py`＋经验接地状态表（文献对照 14 项：IMU 差 1–2 量级、积水实为冰面、叉车慢 2–4 倍为最大三项，record-only 不调参）
  （参数扰动分岔表；标定优先级：制动/堵转/空载/滚阻/转动惯量先测）。
* 补给标记去实体：经停/补给 0.65 米货箱→地面环＋悬浮块＋光柱（标记是信息不是家具）。
* 视觉诚实：2D 认知层等半径碰撞环（美术放大它不放大，以环为准）；补给标记由
  0.65 米实心货箱改为地面环＋悬浮块＋光柱（标记是信息不是家具，路径穿过标记点
  就不能画实心）。信号塔底座已改桅杆＋实际到达圈（见 §4 第 10 项）。
* 回归：CTest 11/11（内容/任务/感知/模型/底盘/预测差分/打滑检测/地图编辑/地图存盘/试玩/存盘规范）＋ pytest 20/20 ＋ 全表比对。

## 4. 待办队列（按顺序，后项不堵前项）

1. **预测模块独立**（已完成）：`src/algo/predictor.*` 纯函数（走廊反射外推），
   跟踪器只出观测/速度/走廊快照；`test_predictor_parity` 五场景逐字节比对常驻。
   协同转弯预测作为第二个实现待进（Predictor 接口届时再开，单实现不开接口）。
2. **打滑检测（路线已收敛：速度级残差，trigger-count 退役）**：`SlipDetector`（加速度差分）已证伪 parked 作反证据（稳态盲＋蠕行误报，不接线、不调阈值）。已证伪过一条近路：差速编码器读的是车体 twist（无胎面状态），打滑不可观，错位采样只会量到积分 jerk。
   细节见 §5 新教训。
    2a. 状态估计接口冻结＋线性 CV 滤波（test_state_estimator／test_estimator_filter 常驻）：6 维＋6x6 协方差＋戳/延迟/源有效＋保旧/匀速外推/拒估 dropout 策略＋未来戳拒收；协方差不增长——增长模型归滤波片，frame/latency/dropout 语义已含；滤波纯函数（predict＋位置/速度/yaw 标量更新，Q/R 全显式参数零隐藏数；6D-CV 线性无 EKF）：合成真值 RMSE 0.029→0.007、3σ 一致 291/300、dropout trace 0.003→0.687 更新缩回 0.019、拒收/回绕/双跑逐比特全钉。开钓鱼记：in-place 协方差 aliasing（整行快照修）＋Qvv 误写 q·dt²（速度方差 50x 欠账——同一 consistency pin 抓到两条 bug，小 Q 时全隐身）。估计器→slip 生产流闭环已落地（test_estimator_chain，mission 未碰）：生产底盘→量化/噪声流→CV 滤波→holder 戳纪律→残差观测。沙地弹射包络 0.009（滤波收紧了直接真值的 0.065，诚实报告非功劳）；搓擦 yaw 0.215 穿过滤波仍可见（直接真值 0.175，滤波不藏剪切）。消费者仍未接行为。
    2b. RO-2 pocket shadow（test_ro2_pocket_shadow 常驻，mission 未碰）：assess_transit 预测 vs 生产 AckermannDrive rollout（同 twist 参考经驱动 PID＋接触物理），直道/90°弯均 ObservedDeviated（max 0.34/0.39m——评估版瞬时巡航假设 vs 生产从静止 spool-up 的起步滞后；时长按构造相等，滞后体现在空间上，time_bias=0 已注明）；估计器影子（真值流＋0.05s 延迟经 EstimateHolder，Fresh／trace 0.06／conf 1/1.06 随集记录）。 verdict 词表 Observed*，永不 Feasible/Allowed。
    2c. Estimator replay＋latency（test_estimator_replay 常驻，mission 未碰）：A. 三源延迟分布（imu p95 5ms／wheel 10ms／laser 80ms，断言排序不设门）；B. 同轨迹×0/20/50/100/200ms→gap 误差 0/1.8/5.4/8.9/17.9cm，单调且为正（stale-ego 偏后、威胁先过—— planner 高估 margin，乐观危险方向；margin 引用此表，本片不定 margin）；C. 末端 dropout×0/50/150/300ms：预算内 Fresh 带残差（50ms→0.06m 诚实成本），超预算 hold/cv 转 PredictedStale（hold 误差≈dropout×速度，CV 贴真值≈0）、flag 拒估。归属结论：0.3s 起步 bias 是驱动 lag，归 planner-margin／执行模型账，不归 estimator。
    2d. Slip observer（本轮开工，设计先行不接线）：残差型，输入＝wheel twist − body twist（＋covariance＋latency），输出＝slip_ratio＋confidence，不做 trigger count；低速不可观、stale 降级，静默骗人否决。生产流首验已落地（test_slip_observer 内，mission 未碰）：RO-2 直线 0→1.5（mission 等效接触）铺装/沙地稳态 |ratio| 0.0024/0.0064（<0.03 零误报，量化不造假 slip；直行不分离路面——诚实非结论）；slipping 零触发（包络内直行超不过牵引帽，正向生产证据待打滑机动，物理不为造 slip 调参）；蠕行 0.1 真量化信号 150/150 拒估。激励矩阵已落地（同文件内生产流，mission 未碰，只收数据）：急加速水面 max 0.038／急刹 min −0.009／沙地满油门 max 0.065 零 flag／履带搓擦纵向 0.011＋yaw 残差 0.175（tread 剪切在 yaw 通道可见，接触 flag 保持安静——力帽与运动学剪切是两回事）／原地打转 0 可观（阴性对照）。坡起换沙地满油门（sim 无高度场，已声明代理）。交付物是残差包络表（未来 confidence 标定的输入），f(slip_confidence) 不在本片定义。门积累已开工（test_shadow_accumulation，mission 未碰）：signal_hold×3 种子 45 episode 格 [35,5][4,1]，sustained-finite 首样本；RO-2 关零等待未纳入；门报告逾期项已刷新（Ackermann 条目转 ✓、计数 29/29、账本补齐）。Gate B v2 已落地：signal_hold 扩到 6 种子＋signal_watch 3 种子（sus-fin 仍单相位，走廊相关已声明）；门报告附录 F 不确定性证据表 v1（非 budget 更名纪律）；seed 分解常驻。平衡 pass（游戏轨道，内容数字零物理风险）：fast_time 统一规则 pinned_rec×1.15 取整——signal_echo 60→75、return_route 9→14、loose_ground 25→36、long_haul 32→46、sand_steer 27→38（推荐 1/0/0→1/1/0，速度项全修好；energy 项此前已挂，另记 backlog 不扩 scope）；signal_cross clear_bar 0.3→0.6（默认 1/1/1→0/1/1，推荐保持 1/1/1，分离成立）。energy 续作（同规则 pinned_rec×1.15）：signal_echo 1.6→2.0、loose_ground 0.45→0.62、long_haul 0.55→0.72、sand_steer 0.48→0.65，四关推荐 1/1/0→1/1/1，默认不动。tracker 去假 PARK（见 docs/parked/tracker-defake.md）：自研关联/KF/方向界/膨胀全验证有效，但两旗舰遭遇证明死因是规划乐观（正确感知＋竞速误判＋无停止线），修感知只换噪声实现。树已回退到绿（22 行逐比特＋34/33 全过），de-fake 转策略层触发器：stop line＋entry-timing 消费＋wait-spot 立项时按 park 报告复用。
关联 parity 笼子落地（零行为变化）：自研 NN（1.0m 门）与 dynamic_hit 并跑，5 关×双配置×30 种子约 43 万射线，一致率 97.3–99.9%；分歧全是 track 生死时刻——switch 需 slot 制 track＋新生逻辑（设计定，下一刀），换 NN 不够。记一次误报：parity 表 big 行与旧基线差 0.1s/0.1m 而 22 行全同——旧基线采于 dwell 内容变更前，唯一动的就是 big 内容，harness 只读无辜。
dwell 落地（registry 第三种行为）：sim::dwell_state 纯时间函数＋单测（周期/零速/退化 wander）＋plaza 转经停（北角各聊 4 秒）；big 重扫船闸 30/30、分离不动；seed63 主街 1mm 擦碰记已知边缘（5Hz 相位代价，120 种子 119/120，推荐 120/120 全净）。
actor 自描述统一落地（draw_forklift/draw_pedestrian 删除，单 draw_actor＋单 SimAgent 向量＋2D 统一标记，大小即语言）：actor 自带 collision/visual，场景 actor_models 回退（删 schema＋3 文件）；序列化补 kind/collision/visual/dormant（roundtrip 门立功一次）；全表逐比特＋截图逐像素全同。
六问答复落地（仿真语义动一处，其余显示层）：行人跟踪了但显示层按叉车数截断 track（refresh 越界 drop）——修好后 2D 可见＋动态地图可标（2-scan 确认不变）；2D 曾把行人画成叉车——人物图标＋DynamicObstacleSnapshot.pedestrian；激光是机体固连整环快照（固态式，3D 转头纯装饰），加扫描相位扫线标记发射时刻；抖动三源头（生死＋速度噪声被外推放大＋相机目标切换）→外推钳到单扫描周期＋目标迟滞 1s；叉车/行人全换真模型（tools/generate_actor_models.py，glb 门禁过，叉车截图验，工人像素待补）；glb 计数 pin 改关系型（引用必有文件）。
视觉三修（全显示层，仿真零改）：2D 叉车跳动→估计外推到帧时刻＋相机指数平滑（5Hz 步进不再瞬移整帧，静帧难验时间域，如实记代码级）；2D 激光稀→旧显示抽稀 1/4 只画，改全量同数据绘制（截图验扇面）；RO-3 履带假→单黑板条换真总成（胎带＋4 负重轮＋2 导向轮＋裙板＋薄荷条＋7 齿，6 mesh 660 顶点，截图验，3D 忽略 mount 旋转故按机体坐标直摆，长 0.72 对齐 track_length 0.7）。
大镇 48x32 落地（五区＋双车＋三行人＋第二单 big_freight）：环线两轮迭代才干净（共线→垂直穿越→撤出车道，接触 3028→0，教训：等车位会坐在环线上被反复穿）；default 120 种子 1cm 擦碰 2 例→主街限速 0.45 后 120/120，bar 不动（几何返工按 T1）；分离窗 (0.56,0.67]，bar 0.6（车身0.45＋会车余量，并行比穿越要宽），0/1/1 vs 1/1/1；T4 执行轨迹三区实踩 7–45%＋等待 265/36；60 runs 3.9s（雷达×盒子线性，开销无感）；截图验 3D 会车/等待/章节头/2D 车道，行人 2D 环线＋3D 代身像素待验（缺对行人对焦手段，follow-up）。
行人 wander 落地（确定性同 patrol）：sim::wander_state 纯时间环路＋kind 严格解析＋行人免撞停（只吃净空＋pedestrian_contacts 计数）＋2D 环线/3D 青衣代身诚实渲染；21 行老数逐比特无漂移（4 位小数全同），镇子物理数三次一致——记录一次门禁乌龙：goals_after2 的镇子安全列 3/3 是 pin（0.5）落地前的旧数，21 行＋物理数全对，只有该列随 bar 走，已如实记。
镇子第三章开张（signal_town 30x22＋首单 town_freight）：四建筑＋三材质分区＋双巡逻；目标模型统一为有序 GOAL（pickup/complete/summon/recharge，reach/deliver_via 全是特例，全表逐比特一致）；sim 事件句＋编辑器 hint 全进 strings.json（110 键，game/ 字面量清零，截图验逐字一致）；计数 pin 改关系断言（增删内容零手改）；章节头跟关卡走（第三章：镇子/第1节，修了全关显示第一章的错）；首单分离在安全轴（0.35 vs 0.67，bar 0.5=车身半径0.45+余量，0/1/1 vs 1/1/1），时/能按 1.15 规则，文案说清快稳交换；镇子场景科幻级光影（表现层）。
slalom 新图首关已否决（完整回退零残留）：S 弯三盒＋砂石分区几何成立、30 种子双配置零碰撞，但 default 四项全胜（12.7s/0.31/0.27Wh/0.14 vs 20.8s/0.30/0.34Wh/0.40）——短平快赛道莽撞最优，north_meet 式倒挂；B 北沿内收一次迭代未改结构。教训：走线概念需要莽撞受罚几何（长赛道能耗／湿滑失控），短赛道天然奖励莽撞。
UI 字号 pass 已落地（world 11–15→14–18 配合控制卡片 174→192 重排、chapter 按钮 17→19、running 卡片组、garage 卡片 13→15＋garage 整屏滚动修底部截断；prediction/debrief 未动——无截图路径，诚实标注；--garage-view 截图入口新增）。
车库属性行已落地（截图验）：卡片三行（名/槽位 stat/价格），stat 取 behavior 真数（抓地mu/极速/米Hz/决策Hz/陀螺噪声），key 覆盖双向门常驻 validator；forklift 3D 按 radius_m 派生（2D 早已如此）；2D 地面纹理平铺＋扬尘纹理粒子＋巡逻线 chevron；3D 地面分材质补丁（material.texture）。
UI 文案外置＋字号 pass 已落地（91 键零缺失零闲置，截图验逐字一致）：assets/strings.json＋schema＋tr/trf（缺键大声失败）；world 11–15→14–18、chapter 按钮 17→20、running 卡片组＋3（容器行高已算）；字形表自动扫 JSON（新文件自动跟字）；key 覆盖静态门常驻。i18n（en 表＋locale 开关＋键对等门）已立项未开工。
UI 四修已落地（桌面截图逐项验证）：关卡列表 ScrollablePanel（21 关可达，此前固定面板超高）；via 标记去家具化（扁平环＋标签，机器人必经点不再穿箱子）；3D bob 熄火（research_base bob_amplitude 0.016→0，内容旋钮，晃动源）；tracked 2D 履带板（track_length_m 连续带，替代小轮盘，物理仍用所装轮胎）；3D 手写 glTF（hull＋双履带，内嵌 buffer，Y 轴映射经 pose_offset 反推，索引越界 bug 抓现行；robot/track schema 放行 .gltf）。验证方式升级：本机 X＋--screenshot 像素级验证，UI 改动合并门加截图项。16 件无碰撞装饰塞进货架 footprint（物理零碰，pinned 逐字）；地图简单是设计（碰撞体未动），复杂走新图。
abilities 注册表已落地（test_abilities 常驻）：22 条目＋schema＋双门禁 resolve＋progress 跟踪/存盘 roundtrip（含老存档兼容）；22 个 ability 奖励此前静默丢弃，现 registry-gated 大声失败。UI 展示等桌面。底盘工厂注册表已落地（test_player_pack 常驻，mission 改线多种子逐字）：新 drive.type＝实现＋creator＋注册一行＋CMake 一行，静态自注册已否决（archive 丢弃未引用目标）；第三方轮胎纯 JSON 上车完赛（overlay 法）；docs/diy.md 作者手册（JSON 菜单／C++ 菜谱／overlay 工作流／诚实红线）。
Gate 定量 DRAFT 已落稿待签（docs/gate-thresholds-draft.md）：A1 九网格提议＋A2 改写（false-go tripwire 常驻：乐观 60% 偏置翻 Infeasible→Feasible，assess 层无解，防御归消费侧 confidence 门/margin 二选一）；B1 形状＋B2 可见性＋C0 parity 建议签字顺序。
车库底盘栏后端已落地（test_chassis_choice 常驻）：presentation opt-in（默认全关）、robot unlock_ability（ro1 常开）、build.chassis＋owned abilities 验证、mission override、last_builds 持久化；boot 开 [ro1,ro2]（RO-2 完赛已验），其余关卡固定零漂移。TGUI 列等桌面。
* 行为注册表已落地（test_behavior_registry＋test_summon_recharge 常驻，mission 改线全表确定性通过）：patrol/wander/dwell 纯求值器＋字符串分发＋未知 type 构造期抛错；SimAgent 改中性 params（mission 零 if/else）；factory 补未知类型契约；summon/recharge overlay 端到端；--seeds 3 全确定性通过。
Family C 首探 signal_pocket 已否决（完整回退零残留， veto 理由存档）：
  长扫巡逻（10m×0.5）＋穿越路线，30 种子双配置 30/30 成功零碰撞但 sus-fin 仅 1、
  pocket shadow 525 帧 feasible 0（HorizonShort 主导——9m 链超 mission shadow 窗，
  离线 20s 窗存在 feasible 证明是仪器窗差异非几何差异）；平衡行无分离
  （default 27.12/0.670 vs recommended 31.16/0.742，scores 全 1/1/1）且
  knowledge_card 承诺影子给三腿口袋与实测矛盾（内容诚实一票否决）。
  定位收获（保留为 Gate B 证据）：DWA 停车点在 swept 区内是内容动不了的；
  mission shadow 窗是 instrument 预算，加窗追可见性＝调参，禁止。
  Family B 首关 signal_cross 已收编（21/21，add-only）：东区南北向 forklift_b（仅新关激活）＋east_gate→signal_tower 穿越；30 种子双配置 30/30 成功零碰撞（否决门过），持续等待 4.5s＋（新 encounter 家族）；sus-fin 仍空（WaitUnsafe 主导 66/100——DWA 把车停在 swept 区里，内容动不了停车点，pocket 行为才是解，Phase-2 事）。
    2e. 履带差速门（test_slip_shadow，只观测不接线）：差速门守住（终关 0 触发）；履带 upper-bound（真值代 IMU）下指标反直觉——沙地 0 触发（准稳态滑动对加速度差分天然不可见），铺装 long_track 4 触发全在货架拐角 v~0.1 蠕行（量化 chatter，非滑移）。结论：现行阈值同时漏检真滑移、误报蠕行噪声—— redesign 方向是速度级持续量（tread vs 独立体速度），不是调阈值；消费者设计仍是行为门。履带分支已解冻：RO-3 直接量胎面（tread 速度进里程计是设计），差速的不可观论证不适用履带——打滑检测进履带接线是打开的机械片（消费者设计即行为门，另议）。
3. **四分之一车摩擦**：已试装一次并回退——差速胎面状态＋共享接触钳制后，
   终关推荐 1–2/3 成功、安全分 0–3/5（5 种子门），回响/ localization 对比同步漂移。
   诊断：诊断性恢复旧增益只修好平静关，终关仍坏——病根在胎面物理本身
   （里程计误差 0.30→0.95，诚实但关卡是按完美编码器调的）。
   裁决：差速退回力钳制（已验证零漂移）；接触模块的牵引钳制保留
   （阿克曼独享，测试全绿）。重做条件：把“终关遭遇重调”立为同一任务
   （内容协同设计：诚实传感器下的等待/净空/配速），不再纯物理单上。
   线束已就绪（`--seed`＋`--seeds K`＋确定性自检＋C++ 种子断言）。
   首份 3 种子证据：终关推荐 2/3 成功（62–80s）、回响推荐 2/3 成功 1/3 安全分，
   其余关种子不敏感。物理改动必须过多种子门，不再钉单种子。
4. **阿克曼出战内容车**（已完成）：RO-2（`robot.ro2`，base 0.62/δmax 0.55/伺服
   0.35s·2.0rad/s）＋关卡按 presentation 选车＋工厂/里程计/遥测/渲染器全分支＋
   认知层最小转弯半径双圆＋第十关“标定新车”（车库→车间出口→标定点，默认丢安全、
   推荐拿安全＋节能）。教训：角落停靠点＋无原地打转＝ docking 必卡，
   转向关的终点必须放在开阔直道上（tolerance 1.0 米也是内容让路）。
   车库“底盘”栏（玩家自选车）未做，记入下一辆车的工作；3D 前轮转向动画已补
   （连杆内外角，2D/3D 同源）。
5. **履带底盘**（已完成）：RO-3（`robot.ro3`，双履带滑移转向＋偏航搓擦阻尼＋
   胎面读数编码器）＋第十一关“履带急转”（车间出口→玻璃门→标定点，默认 0/0/0、
   推荐 1/1/1 横扫）。附带修了终局 docking 控制器：航点记账跑在到达判定前面时，
   按距离停靠而不是原地停车等超时（全底盘受益，差速从未触发不代表逻辑对）。
   首秀关按里程计/时间/能耗评分，净空地板为接触（两边同刷货架到 0.0，是 DWA
   切弯的系统性动作，与风格无关——记为 DWA 调参债，不是物理债）。
6. **地面材质库**（已完成）：`assets/surfaces.json`（铺装/砂石/沙地/积水）＋
   地图矩形只引 id＋`active_in` 门（仿真双入口与双渲染器全过滤）＋第十二关
   “松软地面”（东房间砂石＋沙地，默认 0/1/1、推荐 1/0/0）。迁移零漂移
   （22 行全表逐字一致）。教训：新材质不上图就是死内容，库和关必须同一片交付。
7. **内容编辑器**：round-trip 地基已完成（全仓 43 文件规范化为
   indent=2＋raw UTF-8＋尾换行，逐文件 parse 相等已证，`test_content_roundtrip`
   常驻 CTest；手写与工具存盘从此逐字节一致）。客户端编辑模式 v1 已接线：
   2D 认知层按 E 开关（`MapEditor` 纯逻辑＋单测，拾取/拖移/钳制/存盘全覆盖），
   S 存盘写文件＋全量 reload 校验＋自动重载关卡；只移不动（无增删/缩放，
   存盘保 id 全集，解锁链不断）。交互真机验证（键鼠＋overlay 目视）记入桌面验收；
   v2 已交付：N/B/M 新建障碍/地面/标记（物理默认仅本关可见，地面默认铺装），
   角手柄缩放（0.2 米最小边），Del 删除（删被引用的标记会在存盘校验大声失败）。
8. **20 关批量**（完成：20/20）：第 16 关“沙地转向”（同松软路线换 RO-2：
    沙×转向，默认 5/5 丢安全（里程计 0.41~0.95），推荐 5/5 拿安全（0.07~0.20），
    bar 0.3 两边 margin 0.1；默认 0/1/1、推荐 1/0/0）。第 17 关“湿滑履带”
    （车库→玻璃门短途换 RO-3：默认 5.05~5.23  systematically 迷路但 63 秒能到，
    推荐 1.60~1.67；bar 3.0；默认 0/0/0、推荐 1/1/1 横扫）。
    再否决三条：`north_meet` 北线动态遭遇推荐 5/5 撞叉车（谨慎 timing 系统性撞，
    swift 5/5 通过——完全倒挂，actor 已 revert）；`ice_cross` 结冰对角线
    （巡航无需求力就不打滑，两边里程计 0.04/0.08，冰只惩罚 transients）；
    `long_steer` 长途转向（默认里程计 0.16~0.77 方差 5 倍，混沌 overlapping）。
    动态遭遇新铁律：新巡逻线＋新穿越方向默认按混沌倒挂处理（lost/echo 走廊除外），
    推荐撞＝否决（5 种子，有一撞即否）。冰材质＋冰面已 revert（死内容规则）。第 15 关“沙地履带”（同松软路线换 RO-3：
    沙×搓复合打滑，默认 5/5 超时迷路，推荐 5/5 成功横扫；两边里程计全崩，
    安全只评接触）。否决 `wet_return`（车库→标定点湿对角：短暴露＋噪声主导，
    分布重叠，无 robust 分离）。
    管线先修后扩——修了 `--check` 写盘 bug
    （门禁重写被检查的文件，吃掉过手填 chapter）、补 `chapter` 参数、统一键序；
    加 reach 模板（参数化起终点/槽位/预算/文案/配分、里程计 bar 可选）。
    第 13 关“长途穿越”（信号塔→标定点，默认 0/1/1、推荐 1/0/0，5 种子分离干净）。
    第 14 关“履带爬坡”（同北上路换 RO-3：默认 5/5 撞墙——打滑 17 米迷路，
    推荐 5/5 成功；成功/失败级对比，全作最强）。策展否决两条路线：
    `shelf_cross`（玻璃门→能源站）与 `foldback`/`northbound`（补给↔标定点、
    车间→能源站）——短干燥绕行无 robust 对比：转弯预见熨平走线（净空相等），
    里程计只剩噪声运气（分布重叠），swift 全维度占优。批量铁律：
    每关必须自带对比机制（动态/打滑/长度三选一），否则路线再顺也否决。
    附带关闭前馈遗留的 open 需求：`scrub_climb` 默认就是 robust 碰撞教学
    （打滑迷路撞墙，机制诚实可复盘）。本轮再否决四条：`east_encounter` 东线动态
    遭遇推荐 2/5 撞叉车（谨慎 timing 反而撞，混沌倒挂，actor 门已 revert）；
    `acker_detour` 转向版绕行与差速同线；`tower_run` 全程干燥无分化；
    `long_track` 履带长途默认 150 秒原地摇摆（探针证实 idx 6 倒车迟滞极限环——
    跟随器迟滞按差速调参，履带慢动态下不收敛；失败不可复盘解释，否决；
    附带 revert 掉为救它试过的 docking 无界化）。失败模式必须可复盘解释
    （撞墙可，150 秒摇摆不可）。管线缺删除剪枝（只加不减，删靠手＋
    validator 兜底），记为债。积水叙事债（mu 0.31 实为冰面级，“湿滑”改“结冰”或标 gameplay，待内容拍板）。
    第 19 关“履带长途”翻案（watchdog 泄漏积分修后复测：推荐 careful 一次通过 127.78 秒拿全分，默认 swift 仍在货架拐角极限环 150 秒超时——60 秒走 10 米、倒车占两成；“不可复盘”被推翻为“可定位、可教学”，按 scrub_climb 成功/失败级对比收编：默认 0/0/0、推荐 1/1/1；watchdog 修的是 careful 域，swift 域 flap 仍在，记为已知边界）。
    第 20 关“信号瞭望”收官（sensor 单变量对照：同零件同风格同 seed，long_slow 5Hz 在走廊东口 (17.7,6.1) 撞叉车，short_fast 20Hz 同路 91.72 拿全分；默认 status 3 碰撞 0/0/0 vs 推荐 1/1/1，scrub_climb 式成功/失败对比。机制＝感知刷新率（时间型 staleness），非等待时机、非极限环——新问题。教训：射程 12 米救不了 5Hz，看得勤才走得准。）
    因果诚实补丁（评审纠正）：2×2 解耦（零件×风格）证明 careful 与抓地胎各自必要、各自不充分（eco＋careful 挂、grip＋swift 挂、传感器无关）——教学只认组合因果，不认单参数因果；文案已按 2×2 重写。
    关卡设计规范（路线×相位×构建，评审冻结）：路线定几何约束；相位（seed）定行为分叉，须附轨迹证据；构建（含代码版本）定演化，同 seed 跨构建不保证复现。三者是选择器，不是保证。第 20 关选关标准（实现冻结，先筛候选）：①教学增量（非 18 时序等待、非 19 极限环的新问题）；②对照清晰（路线/seed/构建可解释成功失败差）；③证据完整（构建、seed、坐标、里程、耗时、倒车比、最小间距、得分）；④复现可靠（同配置行为一致，分叉如实记录）；⑤回归不退化。
    第 18 关“信号守望”（18/20，echo 模板：同走廊 seed 42 相位——叉车封路数秒，默认 echo-rec 配置 79.60 秒贴身通过 min 0.153 丢安全，推荐 lost-rec 配置 91.72 秒等过去 17 个等待、maxwait 2.8 秒、min 0.378 拿全分；默认 0/1/1、推荐 1/1/1）。机制＝持续走廊封锁（时间型）vs 终关擦身（空间型）：creep 蹭不过去、必须等秒级窗口。种子即相位选择器（30 种子扫描：成功/撞双吸引子，s42 2.8 秒等待＋成功）。
    第 20 关“信号瞭望”收官（sensor 单变量对照：同零件同风格同 seed，long_slow 5Hz 在走廊东口撞叉车，short_fast 20Hz 同路 91.72 拿全分；默认 status 3 碰撞 0/0/0 vs 推荐 1/1/1。机制＝感知刷新率 staleness，新问题。20/20 达成。）
9. **弯道感知前视**（已完成）：`corner_speed_cap`（转角/腿长→v≤√(a_lat·R)，
   抓地取预览内最小轮胎×地面 mu 的一半）＋三处速度钳位（跟随/DWA/仲裁）。
   证据：默认终关 0.025→0.230（仍丢安全分）、回响默认同；推荐终关 5 种子逐比特
   不变（与 stash 对照验证）；其余结论全不变。看门狗保留为最后防线。
10. **信号塔视觉债**（已完成）：底座改桅杆（R 0.16，非实体语言）＋地面画实际
    到达圈（读关卡 tolerance，不是固定数）。纯视觉，零基线。
11. **第四底盘（裁决：swerve 独立转向轮，暂缓）**：麦轮要建模滚子，独立转向轮
    直接复用车轮/接触模型（每轮独立转角＋共享接触），还能横移＋打转——是更好的
    第四底盘候选。但 3 自由度指挥栈的门槛与轮子类型无关：`DiffCmd` 加 vy、
    跟随/DWA/手动/快照全扩横向，这才是真正的 gate。前提条件：横向规划先行。
    物理模块先行也无意义（开不了的内容是死内容，见第 6 项教训）。
13. **轮速环负载前馈**（已试装并回退，parked，非 dead）：`feedforward_pwm`
    （负载＋扭矩曲线，PID 只修残差）＋三底盘接线。证据：hot  build 5/5 碰撞
    （-0.001..-0.006）→ 5/5 近距离通过（+0.25）——旧碰撞是积分滞后诱发打滑的
    毫米级分岔，前馈让轮胎不抱死，停得更短（类 ABS 道德）。但代价超标：
    localization 推荐 5/5→0/5 安全（里程计 0.016→0.04~0.06，bar 0.025 系统性
    击穿），终关推荐 3/5→2/5（48 号种子翻转），回响/归途余量同步变薄。
    裁决：理论正确，结论保不住，回退（工作未合入，仅留证据）。
    为什么撤的不是正确性：内容是按旧执行器调的，物理单边上线＝调分；
    留物理必须配内容协同设计（新 bar、新失败遭遇、全表重标定），那是独立任务。
    实验室秤修准了是好事，但旧标定不重标就发，发出去的是错的。
    需要性评估（结论）：游戏不需要（无它结论全成立）；sim-to-real 要（前馈可迁移，
    积分器不可），但 sim-to-real 本身 post-MVP。触发条件二选一即重做——
    (a) 真车后端立项（届时前馈/计算力矩是必做项）；(b) 有人认领内容协同设计
    （localization 线重推＋新失败遭遇＋全表重标定）。
    第二轮 verdict（分层后）：运动学层保留（早就在生产路径，见第 14 项）；动力学 FF-PWM 整层关闭——映射正确但净收益为负（积分减半、tracking 不增、饱和上升，estop settle 恶化），mission 侧 shadow 计算已拆除，纯映射＋灵敏度证据 parked 带 verdict 常驻，重试须先推翻 verdict。
    附带关闭：P0 碰撞教学之死——`scrub_climb` 默认以打滑迷路撞墙关闭了该 open 需求
    （机制诚实可复盘，见 §4 第 8 项）。
14. **统一运动学接口**（已完成）：每底盘一对正逆解（差速现成一对不动；
    阿克曼 `steer_for_twist`/`yaw_for_steer` 抽出，表达式逐字搬）＋
    `forward(inverse(x))==x` property test（差速精确，阿克曼限 |v|≥0.5，
    超限报 infeasible）＋mission 里程计委托。纯重构，24 行全表零漂移，
    多种子稳定。不碰动力学（前馈仍 parked），不碰规划器。后续确认：正逆解已是底盘基础能力（drive／odom／ackermann 执行／码盘 yaw／caps 分支全在生产路径），test_inverse_consistency 常驻守边界（极限往返、0.5 地板 intentional 非逆）。
15. **灵敏度分析＋标定流程＋sim-to-real 里程碑**（进行中）：`docs/sim2real.md`
    落盘（缺口清单＋log-and-compare 表格式＋标定六步＋真车 profile 模板）；
    `tools/sensitivity.py` 落地（整份 assets 拷临时目录，逐参数 ±10%，固定种子，
    输出状态/用时/净空/里程计/能耗分岔表；仿真代码零改动）。样本：
    motor_trial 对全部 10 个参数 ±10% 纹丝不动（短平关，预期内）；
    终关全量扰动正在跑，结果回填本条。
    终关结果（±10%，固定种子）：翻转状态的参数——制动+、滚阻±、堵转力+、
    空载±、转动惯量-（7/20 翻转成功→碰撞）；只吃余量的——车重±（净空-0.42）、
    转动惯量+（-0.55）；几乎无感的——摩擦±、滑移±（里程计±0.03）、陀螺噪声/零偏
    （0.0000：无打滑时融合权重全在编码器，陀螺零贡献——诚实，不是 bug）。
    形态：所有翻转收敛到同一碰撞分支（用时-22%、净空-0.62），遭遇只有两个
    吸引子（79.84 秒成功 / ~62 秒碰撞）；凡是让车变快的参数都掉进碰撞分支。
    sim-to-real 标定优先级：制动减速度、堵转力、空载转速、滚阻、转动惯量
    必须实测；摩擦/陀螺噪声在本遭遇可后排（饱和不触发时无贡献）。
16. **MotionTarget 规划器槽**（横向规划到来时）：`BodyTwist{vx,vy,w}`＋选中轨迹＋
    能力协商（规划器 holonomic ⇄ 底盘 accepts_vy，mission 大声失败兜底），关卡按名
    选规划器（与 `presentation.robot` 同构：内容点名，代码备货）。DWA-lite 首个入槽
    （vy=0，non-holonomic）。不提前写代码，设计已冻结在此。
    Wait-aware planning 分拆开工（行为层，不动 DWA，不动底盘）：
    a. **相位入场 t_dep（本轮开工）**：沿路径按巡航速度展开自车未来位形，
       对威胁预测帧扫出最早的安全出发时刻（全程间隙 > margin）；t_dep==0 就走，
       否则等到点，每周期重算不锁死。回答"等多久"给数字，不给 latch。
       输入全现成（预测帧＋路径＋巡航），只加一个时间算子。
       验收：pinned 全表逐字＋east/north 推荐修好＋等后恢复可观测；
       任一不过 revert（前三次的 stop-loss 照用）。
       误差预算（margin 的组成部分，解析界定不抽样）：t_dep 搜索量化 0.2s×巡航
       （最大一项，如 1m/s 下 0.2m）、威胁帧间线性化、规划-执行偏差；
       量化项要么细化步长要么计入 margin，不许 definitional 落空。
       放行误差验收用 miss 制（Argoverse 式：|预测-实际|>阈值即 miss），不只报均值；
       实际安全时刻按整轨迹完成定义，不按单帧。
       三态语义 safe/unsafe/unknown（SOTIF Area 3：unknown 不得混入 safe）；
       观察位挂载即 shadow mode（只记不控），门禁一 MIL 纯函数→门禁二 SIL 离线
       闭环回放→门禁三 shadow→最后行为放行。
       门禁二回放矩阵已钉（test_departure_replay）：nominal 准时出发保 margin、
       保守偏置安全但 miss（多等）、对抗偏置吃光 margin（miss，无碰撞）、
       迎面穿等待位初判 inf 且回放验撞——Phase-1 不认领该场景。
       门禁三 shadow 已挂载（只记不控，隔离经多种子实验验证零行为漂移）：
       inf 带诊断原因（GoNow/Wait/BlockedInWindow/WaitUnsafe/HorizonShort/Invalid，按紧急度排序，
       等待位危险优先）；终关实测 772 帧评估、11 个等待 episode，
       微等待多为 WaitUnsafe（擦身几何）、一次 Wait→GoNow 倒计时。
       数据积累阶段（评审 order：完整性→分布→误差→标定，不跳步）：
       episode 已结构化（ShadowEpisode 与事件 1:1 配对、字段范围、时间戳单调
       全部有断言）；终关分布 first-reason：WaitUnsafe 7、BlockedInWindow 3、
       Wait 1——微等待在两种诊断间闪烁（遭遇几何以 0.1 秒粒度演化），不平均化。
    b. **安全让位**（a 落地后）：采样路径附近空闲格，用第 14 项正逆解验证
       到得了/停得住/并得回，三者全过才列为候选，与原地等/直接走统一评价。
       接口先行冻结（评审交付物，不接行为）：`LocalCandidate` 五型
       （Proceed/WaitInPlace/WaitAtPocket/Rejoin/Detour）＋`check_candidate`
       统一返回 Feasible/Infeasible/Unknown＋原因＋最小间隙证据；时间可判的
       两型已实现（Proceed 即走now才算过，早走也判 Infeasible 保因），空间三型
       诚实 Unknown（kUnassessed，不借用他因）。
       分层归位：DWA 留任局部速度规划＋执行（不动），Phase-2 是行为层候选优选，
       不是第二个规划器进程。
       场景矩阵固定（test_departure_replay）：东线类（WaitUnsafe 反例常驻真关
       episode）、headon-wait-overrun、对抗偏置、horizon 不足，＋正向锚点
       pocket-positive（立即走间隙 -0.33 必撞，侧向 2 米口袋安全等 2 秒，
       下降汇入全程间隙 0.52 过线）。数据继续积累，行为继续冻结。
       评估原语 `timed_gap` 已备（定时自车轨迹 vs 威胁帧， constant-speed 与
       predicted_gap 逐字等价已钉，hold-then-go 口袋雏形已钉）：口袋-等待-汇入
       三腿评估用它，不用 constant-speed 近似。
       下一片（空间最小实现，纯函数，不接 mission）：按腿组合 WaitAtPocket
       可行性——transit 腿复用 A* 路径＋swept 检查＋第 14 项曲率/caps 裁决，
       落地前 transit 腿保持 Unknown 并指名；wait/rejoin 腿用 timed_gap。
       Rejoin 不单列实现（pocket 候选第三腿），中断条件＝每周期重算＋腿 verdict
       翻转（行为层状态机等 gates 全过才写）。
       0.52 账本：回放未建模轨迹采样/跟踪/预测三误差，该数纯模型内——正向锚点
       是存在性证明，不是安全声明；扰动联合注入是放行前置 harness 升级。
       三腿纯函数已落地（test_phase_legs，mission 不动）：assess_transit
       （诚实 Unknown＋名义时刻链）、assess_hold（整区间扫掠，零等待退化为点查、
       超窗 Unknown、先离后返与端点瞬时全捕获）、assess_rejoin（链式 delay、
       hold 前缀分类 WaitUnsafe/Blocked）；assess_pocket_wait 组合裁决
       （任一 Infeasible 则 Infeasible、否则任一 Unknown 则 Unknown，三腿诊断全保留）。
       时间链冻结：后腿起于前腿名义完成、关节瞬时归属两侧（闭区间 min 幂等）。
       Transit 纯函数已落地（test_phase_legs，mission 不动）：A* 只产几何路径
       （调用方膨胀栅格是生成侧契约）；swept 覆盖执行轨迹本身——spin 车走
       turn-drive-turn（原地转＋直行，格子折线的转弯时间如实计入，曾抓到测试按
       直线幻想配窗过紧）；Ackermann 无曲率执行模型（caps 未覆盖）诚实 Unknown；
       A* 无路判 kStaticBlocked（几何已证不可行）；证据带 exec_traj 时间轨迹。
       组合 assess_pocket_wait 同步 map 化：首个 overall Feasible 的 pocket
       （transit 2.9s 零转弯＋hold 2s＋汇入，gap 0.67）。
       下一片：pocket/merge 采样（候选生成）；平滑＋重检随 Ackermann 曲率模型来。
       采样已落地（test_pocket_search，mission 不动）：生成/评估/证据三层分离，
       预算全显式（offsets/window/merge 距/上限，无推导项）； verdict 三态
       （FoundFeasible/AllRefused/Incomplete——截断与 Unknown 永为 Incomplete，
       采样 miss 不证伪空间）；拒绝理由全保留。
       首个 Feasible 复核：chain_gap＝三腿最小（rejoin 段绑定，0.50），
       rejoin_end 11.09≤12（转弯＋hold＋汇入全覆盖），Feasible 要求三腿全过——
       名义模型内有效，不继承给扰动场景。主搜索分布 feasible 3/infeasible 4/
       unknown 1（8 上限截断），拒收含 WaitUnsafe 在 lane 上的口袋。
       门禁审计制已生效（docs/phase2-gate-report.md 首份报告）：逐片自动验证继续，
       人工只进架构分叉/量化标准/门禁完成/stop-loss；新增测试须映射冻结要求、
       已发现缺陷或结论级风险，否则进 backlog（防增厚）。
       联合扰动 harness 已升级真值闭环：估计偏置进估计、自车噪声进真值轨迹
       （规划盲）、同轴重算——首轮分布：pocket 快偏置 0.5006 哨兵、慢偏置 time
       miss、0.1 摆动存活、符号反转等待点真撞。
       空间 shadow 已挂载（Gate B 观察启动）：仅等待周期、只读、stride 去截断
       后终关 184 帧 refused 138/incomplete 46/feasible 0，三腿原因
       WaitUnsafe 1336/HorizonShort 904/Clear 354/Blocked 106；`--seeds 3`
       与挂载前逐字一致。审计账本见 gate 报告 E 节。
       估计误差首测（test_estimation_error，零行为改动）：终关近场速度误差
       均值 0.31/max 1.92、位置均值 0.09/max 0.22；当前 30% 扰动≈观测均值，
       max 已钉压力哨兵（0.43 miss）——0.5 门槛覆盖缺口实锤，预算仍未定义。
       逆解算（FF）上车评估（用户问，结论：公式＋离线可开，实机不行）：
       信号审计（test_signal_quality，零改动）：执行器侧轮速误差均值 0.011／max
       0.09（干净——跟踪器的 0.3／1.9 不进 FF，R(v) 用测量轮速）；微分器噪声
       std 0.27／max 3.79 m/s²，折 PWM 为均值 1％／std 1.5％／max 21％尖峰——
       FF 必须带滤波，滤波滞后吃 T_lead 增益，净收益待重算。
       三个上车 blocker 不变且更硬：①全表针尖化（0.5006／0.230／8mm），FF 必改
       transient＝全表重定基线＋item13 内容协同，非 drop-in；②T_lead 无着落
       （带宽倒数≠有效时延），降级路径＝灵敏度扫描 ±50％ 仿真鲁棒（待做），
       真车辨识仍是最终依据；③积分器证据要重看（pid_i 分布＋跟踪误差＋饱和，
       单看积分下降不算数）。FF 路线阶段 1-3 已落地（20/20、79.84、多种子逐字）：正逆对子补边界（差速极限往返、Ackermann 0.5 地板 intentional 非逆、履带复用即同码）；FF 映射 lead→轮加速度→R→curve→PWM 全钉（手算 8.9544/36）；shadow 每步算、专用 log 不进快照（终关 3992 样本、max 0.748、零饱和，输出逐字一致）。T_lead 取 1/wc（被助闭环自身时间常数，非执行器带宽——模型里无电机滞后），敏感性±50％＋滤波＋离线回放对比是阶段 4（已跑完：parity 位精确＋replay 保真 0.038，T 全程稳定无 blowup，滤波基本不改变结论）。verdict 是诚实的阴性：FF 把积分器卸载约一半（pidrms 0.044→0.021），但 tracking 无改善甚至微降（rmse 0.199→0.202，estop settle 1.76→2.9，饱和 9.6％→11–13％）——transient 本来就饱和主导（整定好的 PID 已打满），FF 加的多是饱和 demand，急停还和停车打架。阶段 5 继续关闭：上车无证据，只有反证据。分层 verdict（评审纠正，我此前的“FF 路线”表述含混，照此拆）：运动学 IK/FK 是底盘基础能力——早就在生产路径（双轮对子进 drive／odom、steer 对子进执行＋码盘、caps 分支），e8ec0b1 只是补了边界锁（极限往返、0.5 地板 intentional 非逆、履带复用即同码），保留并常驻；动力学 FF-PWM 整层关闭——映射正确但净收益为负（积分减半、tracking 不增、饱和上升），mission 侧 shadow 计算已拆除（热路径不留死代码），纯映射＋灵敏度证据 parked 带 verdict 常驻，重试须先推翻 verdict 而非重新发现。一致性≠有效：scrub 力矩未建模、舵机 P-only、无死区，均已写进 honesty 边界。
       等待分型（test_release_error 2x2）：micro-inf 72／micro-finite 9／
       sustained-inf 4／sustained-finite 0——内容里不存在持续等待＋预测生效的
       格子。结论：micro 归 DWA＋creep（已安全，只监视不优化）；Phase-1 行为
       的标定集在现有内容里是空集——需要能产出持续等待的遭遇（内容任务），
       不是更多分析。release-error 只在 finite 格有意义，inf 格只计数。
       Ackermann transit 提案（架构分叉，待中继拍板再实现）：执行模型可全部
       从现有 JSON＋第 14 项对子导出，无需新数据——曲率上限 κ≤tan(max_steer)/L
      （RO-2 约 0.99/m），曲率变化率受 servo rate／v 约束，0.5 m/s 以下禁转
       （policy，非运动学）；A* 折线用 κ 下限圆角 fillet 平滑后对最终曲线重做
       swept（证据不继承）。拍板即开工，不提前写代码。
       放行误差首测（test_release_error）：85 episode／76 初值 inf，9 有限误差
       全负（DWA 蠕行释放≠shadow 预测——口径已声明为 shadow-vs-实际偏差）；
       顺手修了 episode 漏报（无效 DWA 分支此前不调 update，等待可能无记录）。
    c. **综合＋MotionTarget 槽**（横向规划到来时）：等/哪等/绕行一个评价函数，
       安全硬约束先行；DWA-lite 首个入槽。
    Wait-aware planning 细化（外部评审＋我方证据对齐后冻结，同样不提前写代码）：
    时空联合规划，DWA-lite 留任局部运动核心，显式规划等待位置＋进入时机。
    三个决策一起做：在哪走（前进/绕行/让位）、什么时候走、（不安全时）在哪等——候选必须是完整可执行局部行为（正常通行/减速停车/让位后等/通过后进），
    而不是瞬时速度。
    时空碰撞检查查占据区域（整条执行轨迹 vs 障碍物各时刻占据），不是中心点距离；
    执行一小段就重规划，不把一次预测当永久承诺。
    等待位置必须安全且可达可离：原地等只在威胁不扫过本体时合法（东线 8mm 反例
    在案）；pull-aside 不是横向挪一点——对 vy=0 底盘必须用真实运动学约束
    （第 14 项正逆解对＋caps）验证转向/移动/停车/并回全程。
    Ackermann transit 已落地（纯函数，未接线）：A* 几何＋Rmin 圆角（R＝L/tanδmax）＋平滑轨迹静态复核＋伺服滞后自行车跟随，扫的是 SIMULATED 轨迹并二次静态复核；拟合不上／蹭墙／需求超限判 kCapsViolated（证据保留），caps 缺失判 Unknown。22/22、pinned＋多种子逐字。
    `predicted_clear_s` 复活为轨迹评价输入之一（不是放行开关）；不确定大时留
    margin，不机械恢复。安全是硬约束（现有 DWA 碰撞即排除已一致），效用只在
    安全候选中比。
    三阶段（顺序重要）：1 相位入场（不挪等待位置；验证场景必须选等待本身安全的
    走廊——东线几何在此阶段无解是已知结论）；2 安全让位（可达＋停留＋并回全约束）；
    3 综合规划（等/哪等/绕行统一评价）。活性在我方简化为等死检测（单叉车开环巡逻，
    无互等死锁；超时即现有失败语义），不引入多机器人协商机制。
    验收（除既有回归）：时空最小间距、等待位置安全＋等后恢复、预测/实际放行误差、
    无限等待与振荡检测、规划-执行偏差对安全的影响。
    等待分型（85 episode 2x2：micro-inf 72／micro-finite 9／sustained-inf 4／sustained-finite 0）：micro 归 DWA＋creep（已安全，只监视）；release-error 只在 finite 格有意义；sustained-finite 格在现有内容里是空集——标定集缺的不是分析，是能产出持续等待的遭遇（内容任务，signal_hold seed 42 部分填补：2.8 秒等待但初值为 GoNow，仍非倒计时型）。
    先行试装已回退：`predicted_gap`（自车沿路径推进＋威胁预测帧的最小间隙，
    单测已钉）两次接仲裁门均翻转终关 pinned 种子——多等的每一次都 mistime 遭遇，
    且线性预测在巡逻折返处撒谎（折返观察到之前）。第三次试装（否决式 veto：
    仅预测实质重叠才拦，阈值严格零 margin）同样翻转——结论升级：
    在双吸引子遭遇里，任何改变等待时序的 bolt-on 门都会抛硬币，问题不在门限，
    而在“等”本身不是策略：需要的是把等待做进轨迹的规划（相位感知入场），
    不是门。第四次试装（等待进 DWA 评分：近零速候选按走廊放行时间加效用）
    在产生行为前即回退——解剖发现机制死结：原地等待要求威胁从身边经过而
    不穿过本体，但东线实测是擦身 8mm（中心距 0.83 米＝半径和）；等穿过本体的
    威胁＝等着被撞，效用函数救不了几何。等待必须发生在别处（pull-aside）或
    算准相位入场——都是规划器的活，不再试第五种 bolt-on。
    `predicted_gap` parked 待用，测试常驻。
17. **跟随器看门狗泄漏积分**（已完成）：`reverse_time` 清零改 1:1 泄放——看门狗守的是
    “持续 3 秒倒车麻烦”，抖动清零等于把门拆了（慢动态底盘 flap 永远到不了 3 秒）。
    证据：track_turn 默认 91.20→49.80 秒（同分 0/0/0），其余 23 行零漂移，
    多种子分布全吻合。诊断副产品：边际 LOS 目标抖动 × 倒车迟滞 × 慢打转＝极限环，
    完整机制与 target-flap 假说记在此——后续跟随器工作（前视粘滞？）的现成靶子。

19. **控制分层冻结**（架构评审结论）：Mission／Task → Behavior（Proceed／Wait／Pocket／Detour，候选已冻、未接线）→ DWA（局部速度＋碰撞＋短视优化，不管仲裁／等待策略）→ cmd_twist → Kinematics IK（基础能力，常驻）＋ Constraints（caps／authority）→ wheel target → PID（主控）→ Motor。FF-PWM 关闭有证据。并行两条：Safety net（watchdog／spin recovery／docking／倒车 hysteresis，第一性防线，track_turn 91→49 秒是它救的）与离线评估（telemetry／replay／门禁）。IK＋PID 足够的定语：在当前 ODD 与内容分布内；加速度包络是跑车级（上限 3–6 vs 真 AMR 0.3，见 sim2real §2），换负载／真车延迟／新底盘／真加速度体制另议。sim-to-real 栏是 ❌（无真车），形状像量产不等于量产。
    不确定性归属（冻结）：DWA 不负责通行权、Behavior 不负责动力学、IK 不负责安全、PID 不负责规划、Estimator 不负责业务判断——每层只暴露＋量化自己名义下的不确定性（执行 lag 归 planner-margin／执行模型账，时间误差归 estimator 账，见 2b/2c），跨层借数即架构违规。队列顺序：slip-observer → Phase-2 接线最后（缺的是 future-state 可信度，不是行为逻辑）。

18. **桌面验收 / 远端 CI / 8 人测试**：需真机、remote、真人。

## 5. 已知教训（新人必读）

* 文本等价≠行为等价：一次 tracker 扁平化逐行“相同”却翻转了终关遭遇；
  此后重构必须全表比对（`level_report.py` 前后 diff），不得以“逻辑没变”为由跳过。
* 单种子是烟雾测试，不是验证：终关遭遇对种子/噪声实现混沌敏感；
  结论性断言（默认丢安全分、推荐拿安全分）要有 3 种子证据。
* 内容数字会漂移：improvement 经常以“时间±10%、净空重分布”出现；
  结论不变则更新基线，不追逐旧数字。
* 死脚本不落盘：python heredoc 里 assert 失败则全文件未写，不要凭记忆认定“改过了”；
  改完必须 `grep` 确认。
* 体感信号必须真可观：差速编码器读的是车体 twist 派生的轮速（无独立胎面状态），
  打滑在该信号上不可观——拿它和 IMU 做差分，量到的是步内积分 jerk（急停倒车必误报，
  模型饱和计数却为 0）。传感器算法接线前，先证信号可观；胎面状态随四分之一车来。
* 打滑度量选型比阈值重要：加速度差分只看暂态失配——稳态滑动天然看不见，蠕行量化（0.015 米步长／0.02 秒≈0.77 跳变）反而能攒够 4 步确认。度量错了，阈值怎么调都是在蠕行误报和沙地漏检之间搬运。
* 标记物是信息不是家具：路径穿过标记点（经停收集 1.25 米、送达 0.62 米），
  标记视觉就不能是实心体——补给货箱教训。停靠类视觉一律用地面环＋悬浮物语言。
* 等待/放行逻辑近视（实锤）：放行看当前距离，不看通过窗口内的威胁相位。
  慢车通过时间长，需要更宽的窗口，但逻辑给的是同一标准——探针解剖（north 向）：
  谨慎 build 等待 6 次后撞上（叉车正好回来），激进一次等、1.0 米通过。
  东线三关（east_encounter/east_cross/sand_meet）同病：推荐 0~40% 成功。
  真实世界慢＋等＝安全；仿真倒挂证明这是逻辑缺陷，不是物理——修复方向是
  相位感知间隙接受（预测整个通过窗口，不是当前距离），归第 16 项规划器工作，
  调参修不好。推论：lost/echo 的推荐成功也是几何运气（3/5），同一近视只是
  恰好被走廊几何掩盖。
* 方法纪律里的组数已是 40 组（20 关×双配置）：重构前后全表比对，单种子断言
  外加多种子证据。
* 组合因果≠单参数因果：long_track 2×2（零件×风格）证明 careful 与抓地胎各自必要、各自不充分——教学只认组合，monocausal 叙事一律先跑解耦探针。
* 名义通过≠鲁棒：0.5006 哨兵／0.5015 链是阈值上的存在性证明，双侧钉死（漂移任一方向都响），不继承给扰动场景。
* 重放回放≠生产复现：回放采样率（~0.05 秒）与物理步长（0.02 秒）天然错位，闭环回放只能做保真门（RMSE<0.3），位精确只认直接 parity（生产驱动 vs 镜像，同输入逐字节）。

