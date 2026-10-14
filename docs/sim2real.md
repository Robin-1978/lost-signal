# Sim-to-Real：现状、缺口与第一 milestone

> 状态：接口形状就绪（`ChassisModel`、`LidarBackend`、`BodyTwist` 语义≈ROS Twist），
> 执行层全空。真车到来之前，本文件是唯一的 sim-to-real 交付物。
> 纪律句：架构形状接近量产 ≠ 已达到量产证据。sim-to-real 未通过前，只能称为量产形态架构。

## 1. already 有的缝

- `LidarBackend` + `SimLidar`：传感器 HAL 缝，逐比特契约。
- `ChassisModel`（step/telemetry/caps）：执行器抽象形状；目前只有仿真实现。
- `BodyTwist{vx,vy,w}`（plan §4-16 设计冻结）：消息语义对齐 ROS Twist。
- 确定性＋固定步长：同一命令可复现，是对比的前提。

## 2. 缺的（按堵死程度排）

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

## 3. 第一 milestone：log-and-compare（不是"接上"）

小差速车＋ROS 2 Twist 桥，只做单向：同一组 BodyTwist 同时喂仿真和真车，
并排跑，轨迹差画出来。reality-gap 曲线就是第一张证据表。

记录格式（`tools/` 下的报告与本表同构）：

| 种子/命令序列 | 仿真终点 | 真车终点 | 位置差(m) | 航向差(rad) | 备注 |
|---|---|---|---|---|---|

## 4. 标定流程（真车到货按此执行）

1. **几何**：轴距/轮距/轮半径卷尺量；转角 limit 打满舵量角器读数。
2. **舵机 map**：PWM 微秒→弧度逐点扫，记死区、饱和、回差；`servo_response_s`
   用阶跃响应拟合一阶时间常数。
3. **电机曲线**：台架拉 stall force（堵转电流×扭矩常数）与 free speed；
   `wheel_equiv_mass_kg` 由转子惯量×减速比折算，不许拍。
4. **轮胎**：拖拽实验测各材质 mu（铺装/砂石/沙地/积水四个数进 `surfaces.json`，
   不再是猜）；滑移率-力曲线定 `kSlipRef` 量级。
5. **延迟**：命令→运动端到端延迟，`decision_hz` 与前视窗口按此重核。
6. **回放对照**：同命令双边跑，差值进 §3 表；差值收敛前不谈"迁移成功"。

## 5. 真车 robot profile 模板

复制 `assets/robots/ro1.json`，按下表填实测，其余键保持：

| 键 | 怎么测 | 不许 |
|---|---|---|
| wheel_base_m / wheel_radius_m | 卷尺 | 拍 |
| mass_kg / yaw_inertia_kgm2 | 地磅＋摆锤/估算转动惯量 | 用 CAD 值不验证 |
| motor.stall/free | 台架（§4.3） | 抄电机铭牌峰值 |
| motor.wheel_equiv_mass_kg | 转子惯量折算 | 沿用 3.0 |
| steering.* | §4.1＋§4.2 | 沿用 RO-2 的数 |
| max_yaw_accel_radps2 | 实测打转 recording | 抄 8.0 |
