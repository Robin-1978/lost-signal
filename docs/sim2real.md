# Sim-to-Real：现状、缺口与第一 milestone

> 状态：接口形状就绪（`ChassisModel`、`LidarBackend`、`BodyTwist` 语义≈ROS Twist），
> 执行层全空。真车到来之前，本文件是唯一的 sim-to-real 交付物。
> 纪律句：架构形状接近量产 ≠ 已达到量产证据。sim-to-real 未通过前，只能称为量产形态架构。


## 1. already 有的缝

- `LidarBackend` + `SimLidar`：传感器 HAL 缝，逐比特契约。
- `ChassisModel`（step/telemetry/caps）：执行器抽象形状；目前只有仿真实现。
- `BodyTwist{vx,vy,w}`（plan §4-16 设计冻结）：消息语义对齐 ROS Twist。
- 确定性＋固定步长：同一命令可复现，是对比的前提。

## 2. 经验接地状态表（2026-10，文献对照，record-only）

纪律：本表只记录偏差，不调参。对着文献改物理＝对着 sim 调参同罪——
要改必须走 log-and-compare（真 log 进来逐项比对后再动）。verdict 含义：
✓ 同族 / △ 偏但同量级或方向已知 / ✗ 差数量级 / ? 无来源 / ○ 装饰（代码未用）。

| 参数 | sim 值 | 真值来源 | verdict |
|---|---|---|---|
| 铺装 mu_eff 0.875（0.7×1.0/0.8） | 干混凝土 kinetic 0.6–0.85，轿车胎约 0.72（Wong; engineeringtoolbox） | △ 偏热，贴上限 |
| 积水 mu_eff 0.31（0.7×0.35/0.8） | 湿混凝土 0.45–0.75；冰 0.07–0.3 | ✗ 到冰面级—— gameplay 定位存疑，标 gameplay 债 |
| 沙地 mu_eff 0.48（0.7×0.55/0.8） | 干土路 0.65–0.68 | △ 偏低 |
| 砂石 mu_eff 0.61（0.7×0.7/0.8） | 碎石 0.55–0.6 | ✓ |
| 滚阻 铺装 0.01／砂石 0.025 | 混凝土 0.013／碎石 0.02（Wong） | ✓ |
| 滚阻 沙地 0.04 | 田野 0.10–0.35（沙≠田，量级参考） | △ 偏乐观（低） |
| IMU 陀螺 basic 0.035／precise 0.005 rad/s（0.02s 步白噪声→密度×√dt） | MPU6050 0.005、BMI088 0.014°/s/√Hz | basic 0.28°/s/√Hz≈20× 高端，precise 0.04≈3×（带振动可解释） |
| IMU 零偏 basic 0.02／precise 0.002 rad/s（每 run 常值） | BMI088 lifetime ±1°/s，TCO 0.015°/s/K | basic 1.1°/s 踩 lifetime 上沿（最坏沿），precise 0.11°/s ✓ |
| IMU 加速度计 0.15／0.05（密度×√dt→2140／720µg/√Hz） | MPU6050 400、BMI088 230µg/√Hz | basic ~5× MPU，precise ~3× BMI088（带振动可解释）；且 accel_noise 代码未用 ○ 装饰 |
| 激光雷达 5m/20Hz、8m/10Hz、12m/5Hz，噪声 1% | RPLIDAR A1：12m、2–10Hz、8000 点/s、<1% | ✓ 同族（20Hz×48 线 trivially 可实现） |
| 叉车巡逻 0.55 m/s | 通道 2.2（5mph）、行人区 1.3（3mph） | △ 慢 2–4 倍（sim 等待长于现实；反应时间反之） |
| 自车加速度上限 3–6 m/s² | MiR250 满载 0.3 m/s²（9.5m 才到 vmax） | ✗ 快 10–20 倍——DWA/制动/弯道全活在跑车动力学，不是 AMR 动力学 |
| 自车极速 2.4／巡航 1–2 | MiR250 2.0 max；整机 83–94kg vs 我方 24–30kg | △ 速度同族（+20%），质量差 3 倍（玩具尺度注明） |
| slip_ratio 定义 | SAE J670：(1−ΩRc/V)；x-engineer 加速分支分母 rΩ | ✓ 同族（我方＝加速分支＋0.5 地板 guard；SAE 制动分支另议） |
| 舵机 0.35s／2.0rad/s（≈0.52s/60°） | Hitec 标准舵 0.13–0.20s/60° | △ 慢 3–4 倍（重载转向可解释，非 RC 级） |
| 电机堵转 54N／空载 3.0 | 无来源 | ? 待器件手册 |
| 舵机 0.35s／2.0rad/s | 舵机约 0.1–0.2s/60° | △ 偏慢，需手册 |
| 编码器 2048 CPR | 常规值，量化纯数学推导 | ✓ 无需接地 |

最大四项（按对结论的影响排序）：
1. 自车加速度 10–20 倍——比 IMU 更严重：制动距离、弯道速度、DWA 紧急行为、
   estop settle 全是跑车包络。真车 0.3 量级一到，动力学相关结论（FF verdict
   除外——它本来就是阴性）全部重标。这是 sim 里藏得最深的量级错误。
2. IMU basic 档 20 倍（precise 档 3 倍可留）——localization 线建在悲观 IMU 上；
   precise 档基本可用，basic 档待真数据。
2. 积水 mu 0.31 实为冰面——wet_scrub 等关的“湿滑”叙事按真值应叫“结冰”。
   不改关卡（改了就是调内容），改叙事或标 gameplay 债。
3. 叉车慢 2–4 倍——全部等待时长证据（sus-fin、maxwait、release-error）是
   “慢威胁世界”的数；真威胁更快＝窗口更短＋封锁更短，双向都要重标。
4. IMU basic 零偏踩 lifetime 上沿——localization 分离（默认丢安全 vs 推荐拿安全）
   部分建立在“basic 又瞎又飘”上；precise 档可留，basic 档结论待真数据复核。

## 3. 缺的（按堵死程度排）

1. **执行器真后端**：CAN/串口驱动、电流环对接、急停链、指令超时 failsafe。
   接口在，传输是空的。丢信号舵机抱角是最危险项，真车必须回中＋减速＋硬急停。
2. **传感器真后端**：雷达只有仿真实现；编码器/IMU 后端未做（plan Phase A2）。
3. **实时性**：仿真跑多快算多快；真车要实时调度＋延迟预算＋超时监控。
   `decision_hz=10` 只是参数，没有 deadline 语义。
4. **标定**：JSON 里的数是合理猜测，不是测出来的。流程见 §4。
   优先级（`tools/sensitivity.py` 终关 ±10% 实测）：制动减速度、堵转力、
   空载转速、滚阻、转动惯量必须先测（翻转成功/碰撞）；车重/转动惯量吃余量；
   摩擦、陀螺噪声在本遭遇无贡献（未饱和/无打滑时），可后排。
5. **ROS 2 桥**：post-MVP。真车栈讲 ROS/DDS。
6. **感知 gap**：仿真雷达是已知地图光线追踪；真雷达数据进动态跟踪，没验证过。

## 4. 第一 milestone：log-and-compare（不是"接上"）

小差速车＋ROS 2 Twist 桥，只做单向：同一组 BodyTwist 同时喂仿真和真车，
并排跑，轨迹差画出来。reality-gap 曲线就是第一张证据表。

记录格式（`tools/` 下的报告与本表同构）：

| 种子/命令序列 | 仿真终点 | 真车终点 | 位置差(m) | 航向差(rad) | 备注 |
|---|---|---|---|---|---|

## 5. 标定流程（真车到货按此执行）

1. **几何**：轴距/轮距/轮半径卷尺量；转角 limit 打满舵量角器读数。
2. **舵机 map**：PWM 微秒→弧度逐点扫，记死区、饱和、回差；`servo_response_s`
   用阶跃响应拟合一阶时间常数。
3. **电机曲线**：台架拉 stall force（堵转电流×扭矩常数）与 free speed；
   `wheel_equiv_mass_kg` 由转子惯量×减速比折算，不许拍。
4. **轮胎**：拖拽实验测各材质 mu（铺装/砂石/沙地/积水四个数进 `surfaces.json`，
   不再是猜）；滑移率-力曲线定 `kSlipRef` 量级。
5. **延迟**：命令→运动端到端延迟，`decision_hz` 与前视窗口按此重核。
6. **回放对照**：同命令双边跑，差值进 §3 表；差值收敛前不谈"迁移成功"。

## 6. 真车 robot profile 模板

复制 `assets/robots/ro1.json`，按下表填实测，其余键保持：

| 键 | 怎么测 | 不许 |
|---|---|---|
| wheel_base_m / wheel_radius_m | 卷尺 | 拍 |
| mass_kg / yaw_inertia_kgm2 | 地磅＋摆锤/估算转动惯量 | 用 CAD 值不验证 |
| motor.stall/free | 台架（§4.3） | 抄电机铭牌峰值 |
| motor.wheel_equiv_mass_kg | 转子惯量折算 | 沿用 3.0 |
| steering.* | §4.1＋§4.2 | 沿用 RO-2 的数 |
| max_yaw_accel_radps2 | 实测打转 recording | 抄 8.0 |
