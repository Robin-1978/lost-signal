# 内容与表现层 Backlog

> **版本**: 0.2
> **日期**: 2026-10-27
> **状态**: 设计中
> **基线**: `refactor/clean-rebuild` @ `8fa94cb`
> **受众**: 内部
> **定位**: 新增独立文档，不修改现有文档；与现有 Experience 流程（`lost-signal-experience/1`）和资产/场景契约共存

---

## 1. 概述

### 1.1 文档范围

- 本文档记录基于 `e6e17fd` 逐项核对、并在 `8fa94cb` 上复核行号与 F3 状态得到的内容（策划）与表现（美工）方向 backlog，共 25 项，每项含依据、方案、验收条件；第 4 节给出实施顺序、验收门禁与待定决策。
- 范围：Viewer/UI、Experience manifest 与变量注册表、资产与场景、新玩法（装配、进度层），以及 ROBOT/SIM 核心中与上述项有依赖的部分。
- 非目标：不规定算法改动。审计文档中与内容层无依赖的项（P1-1 至 P1-6、P1-10）仍以 `docs/audit-algorithm-sim-best-practice-2026-10-25.md` 为准，本文档不展开。
- 基线说明：写作时 `e6e17fd` 之后只有 `8fa94cb` 一个提交（碰撞几何线框叠加层，含 `--physics-overlay` 开关），涉及 `src/sim/world/mujoco_backend.cpp`、`src/sim/world/mujoco_backend.hpp`、`src/sim/world/world.hpp`、`src/sim/eval/episode.hpp`、`src/sim/scenario/scenario_session.cpp`、`src/sim/viewer/frame.hpp`、`src/sim/viewer/frame_builder.cpp`、`src/sim/viewer/viewer_main.cpp`、`tests/test_mujoco_backend.cpp` 与 `docs/release-continuation-plan.md`。它使 `viewer_main.cpp` 的行号整体后移 1 至 77 行，是 F3 的前提；F3a 是该提交自身的绘制缺陷。文中行号均取自 `8fa94cb`；A2 涉及的三个 `surface_only` 实例在 `e6e17fd` 引入，`8fa94cb` 未改动 `configs/` 与 `assets/`。

### 1.2 结论

1. 瓶颈是内容吞吐量和表现层完成度，不是引擎。设计文档规划第 0–9 章与五套环境 kit（`docs/sim-game-design.md:53-146`、`:521`），基线只有 3 个 Experience、1 套 kit（14 个资产）、1 个完整的对照闭环。
2. 三个缺陷先修：A1（叉车被程序化圆柱盖住）、A2（视觉道具与目标点、叉车起点重叠）、F3a（`8fa94cb` 的碰撞线框姿态错误）。三者都小而确定。A2 的根因是设计文档 `:364` 的契约（实体模型必须绑定碰撞盒，`surface_only` 只用于地表标线）后半句没有在加载器中落实：全部 `*.scene.json` 共 14 个 `surface_only` 实例，11 个是 0.03 m 的贴地件（其中 7 个与车道或目标点相交，属于设计意图），3 个是 0.63–2.27 m 且没有碰撞体的立体件，需要处理的只有这 3 个。
3. 其次是阻塞发布验收或首屏体验的 P1：B1–B5、C1–C2、D1、D4、F1、F3。
4. 新玩法（E1 装配、E2 进度层）排在 C1 与 F1 之后。机械臂（F6）在战役章节中没有对应内容，后置。
5. 条目数量：P0 共 3 项，P1 共 11 项，P2 共 8 项，P3 共 3 项。
6. 实施方式：Viewer 条目沿同一条线串行（依据中以 `viewer_main.cpp` 行号为改动点的条目共 10 项，该文件 5707 行，是 `src` 中最大的源文件），非 Viewer 条目沿另一条线并行；设 G1（视觉）与 G2（内容）两道验收门禁；规模估算 44–64 个提交，不含评审后再定与后置条目。详见第 4 节。
7. 开工前有三件事需要先定：A2 规则放在哪里（决策 1）、三处现有重叠怎么处理（决策 2）、D4 的资产来源事实（决策 3），见 4.4。
8. 依据均可在仓库或附录 B 的复现步骤中核对；未找到证据的地方列入第 6 节。

### 1.3 约定

- 优先级：P0 为缺陷或一致性问题；P1 阻塞发布验收或首屏体验；P2 为内容与表现扩展；P3 后置。
- 条目状态：除 F3（部分完成，碰撞几何线框已在 `8fa94cb` 提交）外均为未开始。
- 规模：S 约 1 个提交，M 约 2–3 个，L 约 4–6 个。这是估算，不是证据（4.6）。
- 行号取自 `8fa94cb`；设计文档行号取自 `docs/sim-game-design.md`（`8fa94cb` 未改动该文件）。
- 截图为本地生成（`build/_shots/`，被 git 忽略），复现方法见附录 B。
- "需要验证"表示仓库内未找到证据，汇总于第 6 节。

---

## 2. 现状基线

| 维度 | 设计规划 | 基线现状 | 证据 |
|---|---|---|---|
| 战役章节 | 第 0–9 章 | 3 个 Experience：`first_contact`、`safety_unknown_obstacle`（第 1 章第二任务，`sim-game-design.md:533`）、`slam_loop` | `configs/experiences/` |
| 单变量对照闭环 | 后续章节复用首章流程（`:532`） | 仅 `first_contact` 含 `intro` 与 `experiments` | `configs/experiences/*.json`，中英文一致 |
| typed 变量注册表 | 按第 2–9 章逐章接入（`:534`） | 1 个变量 `lidar.scan_period_s` | `src/sim/viewer/experience_manifest.cpp:18` |
| 环境 kit | 五套（`:521`） | 1 套（`research_base`，14 个资产）；11 个世界中 3 个有 `.scene.json` | `assets/models/manifest.json`、`configs/worlds/` |
| 视觉验收 | 720p、1080p、超宽屏无关键遮挡（`:537`） | 窗口固定 1100×750，无缩放与全屏代码 | `src/sim/viewer/viewer_main.cpp:81-82`、`:4495` |
| 进度层 | 星级、知识卡、可分享回放（`:15`、`:200`、`:430`） | 无实现 | 见 E2 |
| 资产来源 | 发布前清零来源债务（`:363`） | 53 项中 39 项 `NOASSERTION` | `assets/models/manifest.json` |

---

## 3. Backlog

### A. 缺陷与一致性

#### A1 叉车被程序化圆柱盖住 [P0，已修复]

- **依据**：`viewer_main.cpp:3216-3224`：模块化叉车绘制成功后（`:3221`）置 `drew_modular_forklift = true`，使 `:3224` 的 `if (!drew_modular_forklift && actor_model != nullptr && ...)` 为假；与该 `if` 配对的 `else`（`:3257-3264`，原用于无模型 actor 的程序化圆柱）随之执行，绘制半径 `a.radius_m`、高 `h` 的圆柱，非行人的颜色为 `:3197-3198` 的 `{235,130,40}`。`configs/worlds/research_base.actors.json` 中两台叉车 `radius_m=0.45`、`height_m=1.5`，与本地截图里圆柱的尺寸一致。`git log -S drew_modular_forklift` 只命中 `f2e3a5b`；`tests/` 与 `src/` 中没有其他引用 `drew_modular_forklift` 或 `draw_forklift_parts` 的位置，该路径无测试覆盖。
- **方案**：模块化路径绘制成功时跳过程序化 fallback；fallback 只在两条模型路径都未绘制时执行。
- **验收**：（1）research_base 的 3D 截图中叉车只显示模块化零件，没有 0.9 m × 1.5 m 的橙色圆柱；（2）无 worker 模型时行人仍绘制程序化圆柱；（3）把"选择哪条绘制路径"抽成可单测的纯函数并覆盖各分支组合。需要验证抽取的改动量，该逻辑目前内联在 actor 绘制循环中。
- **修复状态**：已在 `4e3a75a` 的基础上提交本子项。`actor_render_policy` 将模块化叉车、完整 GLB 和程序化 fallback 设为互斥路径；`test_actor_render_policy` 覆盖模块化优先、完整模型、定义缺失、行人和全 fallback 分支。Viewer 的 `research_base` 3D overlay 运行验证通过。
- **范围与依赖**：仅 Viewer，不触及 SIM 与 ROBOT；无功能依赖；排期上在 F3a 之后（前后对比截图要在 `--physics-overlay` 下确认碰撞代理与模块化零件对齐，见 4.2）。

#### A2 立体道具借 `surface_only` 绕过碰撞绑定，与目标点、actor 起点重叠 [P0，已修复]

- **依据**：
  - `configs/worlds/research_base.scene.json` 在 `e6e17fd` 新增三个 `surface_only` 实例：`signal_cross_tower_visual` 在 (17.8, 10.0)，`safety_lane_bollards_visual` 在 (18.0, 6.85)、yaw 90°，`supply_pallet_visual` 在 (10.5, 12.5)、yaw 90°。
  - `configs/worlds/research_base.json` 的 `places` 中 `signal_cross_tower` 为 (17.8, 10.0)、`supply_station` 与 `traffic_trigger` 同为 (10.5, 12.5)，分别与前述信号灯、托盘的中心重合；信号灯占地 x∈[17.59, 18.01]、y∈[9.84, 10.16]，托盘占地 x∈[10.09, 10.91]、y∈[11.95, 13.05]，都包含对应的 place 坐标。
  - 导流柱占地（manifest `expected_bounds_m` 经 yaw 旋转）为 x∈[17.86, 18.14]、y∈[6.19, 7.51]；`research_base.actors.json` 中 `forklift_a` 起点 (17.5, 6.3)、半径 0.45，其中心路径到该占地的最小距离为 0.36 m，小于半径，重叠 0.09 m。
  - 三个实例的高度（`expected_bounds_m` 的 z 范围乘 `scale[2]`）为信号灯 2.27 m、导流柱 0.91 m、托盘 0.63 m。全部 `*.scene.json`（`research_base`、`safety_lab`、`slam_loop_lab`）共 14 个 `surface_only` 实例，其余 11 个高度均为 0.03 m：`maintenance_bay_pad`、`forklift_lane_north_01`、`forklift_lane_north_02`、`forklift_lane_south_01`、`forklift_lane_south_02`、`start_pad`、`exit_marker`、`home_pad`、`east_marker`、`north_marker`、`west_marker`。
  - 设计文档 `docs/sim-game-design.md:364` 写明契约：实体环境模型必须绑定 world 中稳定且唯一的 `box_id`，地表标线必须明确声明 `surface_only`，缺失碰撞体、中心不一致、重复绑定或非环境资产会拒绝加载。`src/sim/viewer/scene_manifest.hpp:53-54` 同样把 `surface_only` 定义为贴近地面、涂装或警示带之类的装饰。加载器实现了绑定、中心偏差与重复绑定的拒绝（`scene_manifest.cpp:180-187`、`:327-345`），但对 `surface_only` 只校验离地不超过 0.10 m（`:168-169`），不校验资产高度，立体件因此可以以 `surface_only` 通过校验：契约的后半句没有落实。
  - `docs/release-continuation-plan.md:243-245` 写明三类资产作为 `surface_only` 视觉实例“放到 `signal_cross_tower`、叉车通道和 `supply_station`”，且不创建碰撞体。重叠是有意摆放，与上述契约冲突，处理方式是策划决定（4.4 的决策 2），不是笔误修正。
  - 对比方案：若规则改为“footprint 与 place、actor 路径相交即拒绝”且不限高度，到达半径取 0.25 m 或 0.30 m 时都是 10 个实例命中，其中 7 个是贴地件：`forklift_lane_north_01`、`forklift_lane_north_02` 压在叉车路径上，`exit_marker`、`home_pad`、`east_marker`、`north_marker`、`west_marker` 放在各自的 place 上，均为设计意图，属于误报；其余 3 个是真正的立体件：信号灯与 `signal_cross_tower`、导流柱与 `forklift_a`、托盘与 `supply_station` 及 `traffic_trigger`。
  - 现有保护：actor 路径对静态 box 的校验在加载 sidecar 时执行，非法文件被拒绝且不添加任何 actor（`src/sim/scenario/actor_sidecar.hpp:17-33`，实现 `actor_sidecar.cpp:48-75`），测试按目录枚举所有 `*.actors.json` 复核（`tests/test_sim_actor.cpp:115-141`）。立体件绑定碰撞盒后，叉车路径与它的相交由这条校验覆盖。
  - 缺口：place 与静态 box 的关系没有校验。`src/sim` 中以 place 配合 inside、overlap、blocked 等关键词检索无命中，`src/sim/eval/world_io.cpp:413` 起的加载只解析 place 的字段；设计文档 `:306` 要求“目标 footprint 可达”，其实现位置需要验证。现有 11 个世界的 77 个 place 到最近静态 box 的距离均不小于 0.30 m（附录 B）。
  - 沙盒下 objective 列表取世界内全部 place（`src/sim/experiment/experience_runtime.cpp:209-214`）；给定 Experience 时只取其列出的 place（`:215-226`）。research_base 的 11 个 place 与沙盒 HUD 的 `obj n/11` 一致。到达容差有两处默认值：SIM 侧验收的 `objective_acceptance.position_tolerance_m` 为 0.25 m（`src/sim/eval/sim_config.hpp:22`，判定见 `src/sim/scenario/objectives.cpp:36`、`:66`），ROBOT 侧 `arrival.position_tolerance_m` 为 0.30 m（`src/robot/platform/description.hpp:60`）。
  - 三个实例均无碰撞体（`docs/release-continuation-plan.md:244` 同样写明），该小节同时写明提交前仍需完成视觉与碰撞代理一致性验收（`:247-248`）。
  - 场景清单校验中，`surface_only` 实例在通用字段校验（资产存在、缩放范围、二选一声明，`src/sim/viewer/scene_manifest.cpp:135-166`）之外，只检查离地不超过 0.10 m，随后直接通过（`:167-173`）；绑定碰撞盒的实例才校验视觉中心与碰撞盒中心相距不超过 0.02 m（`:180-187`）。`parse_instance` 的输入含资产清单与静态 box（`:124-126`），计算资产高度不需要新输入。
  - `8fa94cb` 的碰撞几何线框只绘制活动 link，静态几何不在其中（`docs/release-continuation-plan.md:254`）；本地 `--physics-overlay` 截图中叉车周围有洋红线框，导流柱与信号塔没有，因此这处重叠在 overlay 中看不到。
  - 设计文档 `:306` 要求场景作者校验"目标 footprint 可达、初始无穿透"；审计 P1-9 要求建立 mesh-to-collision 偏差检查。
- **方案**：
  - （a）规则 R1（落实设计文档 `:364` 的契约）：`surface_only` 实例必须贴地，顶面高度不超过 0.10 m，与 `scene_manifest.cpp:168` 的离地界限同值；违反时按现有 `error` 路径拒绝加载。这是单实例规则，不需要 places 与 actors 作为输入，也不改变 `load_scene_manifest` 的签名。当前数据中两类高度相距 0.03 m 与 0.63 m，界限在此区间内取任何值结果相同；顶面高度如何由 `position_m[2]`、`scale[2]` 与资产的 `expected_bounds_m`、`pivot_m` 计算，实施时核对。落点见 4.4 的决策 1。
  - （b）三个立体实例的处理见 4.4 的决策 2：在同一提交内为它们绑定碰撞盒，并移到目标点与叉车路径之外（导流柱让出 `forklift_a` 起点的 0.45 m 半径，信号灯与托盘离开 `signal_cross_tower`、`supply_station`、`traffic_trigger`）；绑定后叉车路径由现有 actor 路径校验覆盖，无需另写 footprint 相交计算；需评估新增碰撞盒对 research_base 任务与测试的影响。实例 id 保持不变。
  - （c）规则 R2（可选加固，不属于 P0 范围）：place 与任意静态 box 保持不小于到达半径的净空，补上设计文档 `:306` 的“目标 footprint 可达”。现有 77 个 place 在 0.30 m 半径下零例外，可落在按目录枚举世界的测试里（做法参照 `tests/test_sim_actor.cpp:115-141`），到达半径作为参数。它能拦住“给立体件绑定碰撞盒后压在目标点上”的情形。
  - （d）把分类写入场景作者规范：实体立体道具绑定碰撞盒并放在目标点旁，只有贴地标线才用 `surface_only`。
- **验收**：（1）`test_scene_manifest` 新增负例：临时场景中的立体件声明 `surface_only` 被拒绝且错误信息指明高度，该负例在规则落地前失败；（2）规则落地后，当前数据中被拒绝的恰为上述 3 个立体件，其余 11 个贴地件不受影响；（3）内容处理与规则在同一提交内落地，提交后 `configs/worlds/` 下所有 `*.scene.json` 加载通过，`test_scene_manifest` 的现有断言（38 个实例、`has_instance` 三个 id，`:45-55`）不变；（4）research_base 相关任务与实验测试通过，新增碰撞盒的影响范围需要验证；（5）viewer 截图中三件道具不压在目标点与叉车路径上；`--physics-overlay` 下视觉与碰撞线框对齐的检查在 F3a 修复后补做，不阻塞规则与内容的提交。
 - **修复状态**：已提交为 `d5f9a69 fix: enforce physical scene collision contracts`。三个立体件已绑定 collision box 并移出目标和 actor 路径；loader 改用场景显式 `surface_only_max_height_m`，同时校验实例基座和按 scale 计算的模型顶部，不再使用隐藏 C++ 高度阈值；`scene_manifest` 与 `world_place_clearance` 通过。
 - **范围与依赖**：场景清单校验、world JSON 与测试；不修改 `viewer_main.cpp`，在线 D 并行（4.2）；无前置条目。

### B. Viewer 与 UI

#### B1 跟随相机遮挡淡出 [P1]

- **依据**：研究基地跟随相机的默认视角下，货架几乎完全遮住机器人（本地截图）；`src/sim/viewer` 中 `occlu`、`see-through`、`spring-arm`、`dither` 及相机碰撞、裁剪相关关键词零命中；设计文档 `:537` 要求无关键遮挡。
- **方案**：相机到机器人的线段与静态几何（world box 与 GLB 实例）相交时，对遮挡物做距离淡出（dither 或 alpha），或缩短相机弹簧臂；只影响渲染，不影响物理与 ROBOT；由 sim time 驱动，保证暂停与回放一致。
- **验收**：research_base、safety_lab 跟随相机默认视角下，机器人可见像素占比达到评审确定的阈值；暂停与回放逐帧一致；Turbo 下计入 12/24 ms 帧预算（`sim-game-design.md:378`）。
- **范围与依赖**：仅 Viewer；无依赖。

#### B2 窗口缩放、全屏与 UI 缩放系数 [P1]

- **依据**：窗口常量在 `viewer_main.cpp:81-82`（1100×750），`InitWindow` 在 `:4495`；`src/sim/viewer` 中 `RESIZABLE`、`FULLSCREEN`、`ToggleFullscreen`、`SetWindowSize`、`IsWindowResized` 及 DPI 相关符号零命中；CLI 参数表中没有窗口尺寸参数（现有参数为 `--3d`、`--assets`、`--camera-dist`、`--camera-pitch`、`--camera-target`、`--camera-yaw`、`--experience`、`--experiment-out`、`--help`、`--history-seconds`、`--lidar`、`--locale`、`--map`、`--mute`、`--physics-overlay`、`--play`、`--quality`、`--rate`、`--robot`、`--robot-evidence`、`--sandbox`、`--scene`、`--shot`、`--sim`、`--turbo`）。设计文档 `:537` 要求 720p、1080p、超宽屏无关键遮挡，该验收项当前没有实现路径。
- **方案**：窗口可缩放并支持全屏切换；增加窗口尺寸 CLI 参数，使 Xvfb 下可复现；引入 UI 缩放系数（按窗口高度与 DPI 计算），`draw_ui_text` 字号与面板尺寸乘以系数；面板布局按宽高比分支（含超宽）。分两步：先系数，后布局。
- **验收**：1280×720、1920×1080、2560×1080 三种尺寸下，首章英雄镜头、简报、运行 HUD、复盘抽屉的截图无裁切与重叠。
- **范围与依赖**：仅 Viewer；B3、B4 依赖本项的缩放系数。

#### B3 字体清晰度与字号阶梯 [P1]

- **依据**：`viewer_main.cpp` 中 46 个字面量字号的 `draw_ui_text` 调用，分布为 11 px×6、12 px×7、13 px×16、14 px×8、15 px×5、19 px×1、20 px×2、23 px×1，另有 2 个非字面量；其中 37 个不大于 14 px。中文字体以 48 px 图集加载（`:187`）并设置双线性过滤（`:191`），缩小到 11–15 px 绘制；首章字幕与简报正文在本地截图中边缘偏糊。
- **方案**：定义字号阶梯（最小正文字号，并随 B2 的缩放系数放大）；字体图集生成 mipmap，或改用 SDF 字体。需要验证：当前 raylib 版本对 SDF 字体的支持，以及 `viewer_main.cpp:123-187` 的 CJK 字符预加载策略是否兼容。
- **验收**：1080p 下 11–15 px 档文字边缘清晰，以前后对比截图评审；字号阶梯表写入文档。
- **范围与依赖**：仅 Viewer；依赖 B2。

#### B4 状态行与按键提示的可读性 [P1]

- **依据**：顶部状态行格式串在 `viewer_main.cpp:5581`、`:5587`（`req=… actual=… step/s p99/max=… ms`），在 1100 px 宽度下右侧被截断（本地截图中末尾的 `view=…` 与运行状态被裁掉）；状态行以 `BLACK` 经 raylib 默认字体 `DrawText` 绘制，字号 14（`:5605`），`:5560-5620` 内没有 `DrawRectangle` 底板，黑字叠在深色建筑与夜景天空上时对比度低（本地截图）；底部按键提示在深色场景对比度低（本地截图）。
- **方案**：状态行按宽度自适应（缩写或换行），精简版与完整版按视图层切换；底部提示加半透明底板。
- **验收**：1100×750 与 1920×1080 下状态行完整可见；底部提示与背景的对比度达到评审确定的阈值（候选：WCAG AA 的 4.5:1）。
- **范围与依赖**：仅 Viewer；依赖 B2。

#### B5 信标与证据层不遮挡机器人 [P1]

- **依据**：目标信标由 `draw_goal_beacon_3d` 绘制（定义 `viewer_main.cpp:2895`，调用 `:3190`）；机器人位于目标信标环上时，信标竖柱、金色线框球与包围盒叠在机器人上（本地截图，小镇场景）；证据文字（`:5607-5618`）直接以 `DrawText` 绘制，两行深紫 `{78,54,132}`、一行青绿 `{24,116,105}`，无底板，激光射线与深色建筑背景穿过文字区域时可读性下降（本地截图）。
- **方案**：机器人到达或接近目标时，信标退为地面圆环加淡出光柱；线框球仅在 Engineering 层保留；证据文字加深色底板。
- **验收**：到达目标瞬间机器人在 Player 与 Learning 层完全可见且无线框球；证据文字对比度达到 B4 的阈值。
- **范围与依赖**：仅 Viewer；对比度阈值与 B4 共用。

#### B6 中文界面中的英文标签 [P2]

- **依据**：首章简报的部件标签在 `viewer_main.cpp:2192-2197`：`tr("WHEEL", "DRIVE")`、`tr("CASTER", "CASTER")`、`"LiDAR"`、`tr("BODY", "BODY")`。中文界面显示 DRIVE / CASTER / LiDAR / BODY，英文界面显示 WHEEL，两种语言对同一概念使用不同的词。
- **方案**：评审确定规则（全部保留英文工程标签，或全部中文化），统一 `tr` 表；`load_ui_font` 的字符预加载表同步覆盖新增中文字符（`:123-187`）。
- **验收**：zh-CN 与 en 截图的标签规则一致，无缺字方框。
- **范围与依赖**：仅 Viewer；无依赖。

### C. Experience 与内容

#### C1 为 `safety_unknown_obstacle` 与 `slam_loop` 补英雄镜头与单变量对照 [P1]

- **依据**：`intro` 与 `experiments` 仅 `first_contact` 含有，中英文 manifest 一致；设计文档 `:532` 要求后续章节复用同一流程，`:519` 要求每章一个英雄画面，`:534` 要求逐章接入 typed registry。
- **方案**：每个 Experience 增加 1 个 `intro`（英雄画面）与 1 个 `experiments`。候选变量：安全章为速度上限，SLAM 章为里程计打滑或噪声强度。需要验证：对应配置项是否存在，以及是否适合作为 typed 变量暴露（见 F1）。
- **验收**：两章通过 manifest 校验与 `EpisodePair` 校验，四项裁决（SIM 有效、安全、任务、实验，见 `sim-game-design.md:395`）可见；对照运行后并排显示耗时、碰撞、最小净空，与首章一致；每章闭环在 60–90 秒内（`:423`）。
- **范围与依赖**：Experience manifest；依赖 F1。

#### C2 预测题改造 [P1]

- **依据**：三章 manifest 的 `choices[0]` 都是最长、最完整的选项（`safe_tradeoff`、`observe_avoid_complete`、`map_to_odom_correction`）；安全章与 SLAM 章的干扰项是稻草人（`ROBOT follows the prior route through the obstacle`、`Teleport odom-to-base to simulator truth`、`ROBOT must stop forever because the prior is imperfect`）。设计文档 `:399` 已禁止 manifest 预写 `correct`，裁决来自实测 outcome；`:430` 把预测错误定义为学习证据。需要验证：新人试玩时的首选项命中率。
- **方案**：（a）选项顺序按 seed 确定性洗牌；（b）干扰项改写为真实的常见误解；（c）增加数值预测题型（净空减少量、耗时增加量），判定复用 `experiments` 已有的阈值字段（`minimum_clearance_loss_m`、`minimum_elapsed_increase_s` 及 equivalent 容差字段）。
- **验收**：manifest schema 对新题型有校验及负例测试；同一 Episode 回放的选项顺序一致；每题至少 2 个干扰项对应真实误解（人工评审）。
- **范围与依赖**：Experience manifest 与 schema；与 `lost-signal-experience/1` 的现有选择题格式共存。

#### C3 首章开场字幕时长与镜头对位 [P2]

- **依据**：`configs/experiences/zh-CN/first_contact.json` 的 `intro.shots` 时长为 1.6 / 1.3 / 1.4 / 1.7 s（合计 6.0 s，符合 `sim-game-design.md:518` 的 5–7 秒），正文 32 / 33 / 40 / 36 字，标题 9–10 字。右上提示为"回车 / 空格 / 点击 跳过"（`viewer_main.cpp:1552`）。转场时镜头先于字幕切换（本地截图帧 40：字幕仍是驱动轮，画面已转到车头脚轮），需要验证是否为有意的镜头提前量。
- **方案**：开场字幕只保留标题（不超过 12 字），长说明交给玩家控速的 hotspot 卡（驱动轮与 LiDAR 已是必看项）；或改为玩家点击推进。
- **验收**：3 名未接触过项目的观察者中，至少 2 名能在开场结束时说出驱动轮或 LiDAR 的作用之一（阈值评审时可调整）；转场附近的帧中字幕与镜头对象一致，或书面确认镜头提前量属设计。
- **范围与依赖**：Experience manifest 与 Viewer；无依赖。

#### C4 第 7 章"底盘不是同一种车"的 Experience [P2]

- **依据**：设计文档 `:128-139`；资产清单已含 `robots.ro2`（底盘与前转向节）、`ro3`（履带）、`ro4`（swerve 模块）、`ro5`（辊轮），缺专属场地与 Experience；`:137` 要求履带与 Mecanum 的简化模型在 UI 标明能力边界。
- **方案**：先做 1 个任务（同一路线，差速对 Ackermann），复用 C1 的单变量对照；底盘选择是分类变量，依赖 F1 的分类变量支持。
- **验收**：两种底盘在同一 world、同 seed 下完成；UI 标明简化模型的能力边界；四项裁决可见。
- **范围与依赖**：Experience、场景与 UI；依赖 F1、C1。

### D. 资产与场景

#### D1 小镇与工业园的场景 kit 与 `.scene.json` [P1]

- **依据**：`configs/worlds/` 的 11 个世界中只有 `research_base`、`safety_lab`、`slam_loop_lab` 有 `.scene.json`；`signal_town` 是发布任务、`industrial_200k` 是性能场景（`docs/release-performance-2026-10-27.md`）；设计文档 `:521` 要求五套 kit，基线只有研究基地（14 个资产）；本地截图中两个世界的地面为单色、障碍为无细节方块。
- **方案**：先为 `signal_town` 建 `.scene.json`，复用现有 14 个资产中适用者（信号灯、导流柱、托盘堆、警示带、墙板等），并按"物流仓、公共大厅、小镇"主题新增资产；新资产沿用 `tools/modeling/build_research_base_kit.py` 的流程并通过 `tools/modeling/inspect_assets.py` 门禁；通用件是否拆出 `environment.research_base.*` 命名空间待决。随后为 `industrial_200k` 建 `.scene.json`。
- **验收**：两个世界各有 `.scene.json` 并被 `test_scene_manifest` 覆盖；A2 的规则（`surface_only` 必须贴地，立体件绑定碰撞盒）对它们同样通过；20 万平方米任务性能不退化。需要验证：`release-performance-2026-10-27.md` 的基线测于第 4 阶段，需先重测。
- **范围与依赖**：资产、场景与测试；依赖 A2。

#### D2 地面贴图是死资产 [P2]

- **依据**：`assets/textures/environment/` 含 `concrete_floor.png`、`gravel_floor.png`、`mud_floor.png`、`sand_floor.png`、`water_floor.png`、`dust_puff.png`。前五张在 `src/`、`configs/`、`tests/` 中零引用（`git grep -F <文件名> 8fa94cb` 统计），viewer 唯一加载的贴图是 `dust_puff.png`（`viewer_main.cpp:668-669`）。世界 zone 的材质名包括 `standing_water`、`sand`、`gravel`、`steel_deck`、`mud`、`industrial_gravel`、`loading_wet`、`paved`、`weathered_asphalt`（`configs/worlds/*.json`）。
- **方案**：建立 zone 材质名到地面材质的绑定（贴图、平铺、对比度），先覆盖 `paved`、`gravel`、`mud`、`sand`、`standing_water`；只改渲染，不改物理材质（摩擦）与 ROBOT 观测。
- **验收**：`signal_town` 截图中地面可见材质细节；物理材质与 ROBOT 观测的回归测试结果不变；5 张贴图均有引用，否则删除。
- **范围与依赖**：Viewer 与资产；无依赖。

#### D3 环境声身份与音频清理 [P2]（已完成，2026-10-30）

- **依据**：`assets/audio` 只有事件音与循环音；`viewer_audio.cpp:29-36` 只加载 8 个 WAV（`collision`、`decision_wait`、`decision_clear`、`relay_restored`、`motor_drive`、`wheel_roll`、`tire_slip`、`service_brake`）；`fruit_damage`、`harvest_pickup`、`power_pickup`、`task_abandoned`、`traffic_warning` 与 `cues.json` 无代码引用；设计文档 `:521` 要求每套 kit 有声音身份。
- **方案**：删除或归档 5 个未引用的 WAV 与 `cues.json`；为 research_base 制作第 1 条环境底噪循环，并登记来源与许可证。
- **验收**：`assets/audio` 的九个 WAV 与代码加载端点一一对应，由 `tests/test_audio_assets` 门禁检查；新增环境循环有 `assets/audio/manifest.json` 与 `LICENSES.md` 来源/许可证登记。未引用的五个旧 WAV 与 `cues.json` 已删除。
- **范围与依赖**：音频资产与 Viewer；无依赖。

#### D4 资产来源债务清零 [P1]（已完成，2026-10-30）

- **依据**：`assets/models/manifest.json` 53 项中 39 项为 `NOASSERTION`、8 项 `CC0-1.0`、6 项 `LicenseRef-LostSignal-Project`；设计文档 `:363` 写明发布资产包前必须清零这项来源债务；音频与贴图没有许可证登记（模型清单之外仅有 `assets/models/LICENSES.md` 与字体的 `assets/fonts/OFL.txt`）。
- **方案**：逐项补来源（自制资产登记为 `LicenseRef-LostSignal-Project`，第三方资产登记实际许可证与来源）；为音频与贴图增加与模型同级的清单。
- **验收**：manifest 中 `NOASSERTION` 为 0，并由 `test_asset_manifest` 作为发布门禁；音频有 `assets/audio/manifest.json`/`LICENSES.md`，模型有 `PROVENANCE.md`/`LICENSES.md`，根许可证与第三方声明已恢复。
- **范围与依赖**：资产元数据；无依赖。

### E. 新玩法

#### E1 装配玩法：把 ro1 的 6 件配件变成玩家可选零件 [P2]（基础切片已完成，2026-10-30）

- **依据**：`assets/models/manifest.json` 含 `robots.ro1` 的 `wheel_eco`、`wheel_grip`、`lidar_fast`、`lidar_long`、`compute_light`、`compute_performance`；这 6 个 ID 在 `src/`、`configs/`、`tests/` 中零引用（`standard` 版由 `src/sim/viewer/asset_manifest.cpp` 与 `viewer_main.cpp` 引用）；设计文档中没有装配或商店玩法（`商店`、`loadout` 无命中），属于新增玩法。
- **方案**：零件映射到可验证的配置变量：LiDAR 型号对应扫描周期与量程，轮对应摩擦与滚阻（SIM 物理参数），算力对应 ROBOT 决策周期或延迟（ROBOT 配置）；选择只写 ROBOT 与 SIM 配置并经 typed registry；渲染按所选零件加载对应 GLB。
- **验收**：（1）`assembly_registry` 的三槽位选择写入 runtime invariant digest；（2）轮胎、LiDAR、算力选择在首次 observation 前分别修改 typed Robot/SIM config；（3）未知零件、manifest 负例和 13 个章节首 tick 有回归测试；（4）新增 `assembly_lab`，现有首章流程不变。后续仍需把装配卡片和星级进度接入存档 UI。
- **范围与依赖**：依赖 F1；评审事项：玩法是否进入战役（设计文档受众表 `:15` 未列此项）。

#### E2 进度层：本地 profile、星级、知识卡、回放导出 [P2]（profile 核心已完成，2026-10-30）

- **依据**：设计文档 `:15`、`:200`、`:430` 承诺星级、知识卡、可分享回放；`src/sim/viewer` 与 `configs/experiences` 中 `star`、`knowledge card`、`share`、`achievement` 无命中；`ExperienceProgressionPolicy` 仅做解析（`experience_manifest.cpp:506-517`）；`next_experience` 是章节链指针；成对 replay 的 v2 bundle 已有（`sim-game-design.md:531`）。
- **方案**：最小版为本地 profile JSON（章节完成、最佳净空与耗时、已看知识卡）；星级只映射 `:200` 规定的公开维度；分享先做成导出成对 replay 与实验报告。
- **验收**：`player_profile` 存取有单测，文件损坏/版本不匹配时回退为空 profile 并返回提示；星级不抵消安全失败；进度数据不进入 ROBOT 与 SIM 判定。Viewer 简报和复盘已显示当前章节星级、最佳成绩与已发布知识卡；profile 默认使用 XDG 用户状态目录，也支持 `--profile FILE`。跨章节目录和更多卡片交互仍属于后续内容批次。
- **范围与依赖**：Viewer 与存档；建议排在 C1、C2 之后。

### F. ROBOT/SIM 核心依赖项

#### F1 typed 变量注册表扩展 [P1]

- **依据**：注册表只含 `lidar.scan_period_s`（单位 s，范围 0.001–10.0，`src/sim/viewer/experience_manifest.cpp:18`）；使用点为 `src/sim/experiment/episode_pair.cpp:60`、`:165`、`:279` 与 `src/sim/experiment/experience_runtime.cpp:181`；设计文档 `:399` 要求未知变量与一章内多个变量在加载时拒绝，`:534` 要求逐章接入。
- **方案**：把注册表改为表驱动（id、单位、范围、固定物理网格约束、所属配置域 ROBOT 或 SIM、应用函数），增加 C1 与 E1 需要的变量，并为 C4 增加分类变量支持（类型设计需要验证）。
- **验收**：未知变量、超范围、一章多变量继续在加载时拒绝（沿用现有负例）；每个新变量有单位、范围、应用点单测与 `EpisodePair` 一致性测试；ROBOT 配置域变量不读取 SIM 真值（`tests/check_boundary.py` 通过）。
- **范围与依赖**：`sim/experiment` 与 `sim/viewer`；被 C1、C4、E1 依赖。

#### F2 动态 actor 与叉车可动 link [P3]

- **依据**：actor 是 mocap kinematic body（`src/sim/world/mujoco_build.cpp:1188-1190`、`:1206`）；叉车 link 通过 `add_actor_visual_link`（`:1191`）作为固定子坐标系，注释说明后续可加升降与倾斜关节（`:1237-1238`）；审计 P1-8；设计文档 G6（`:411-415`）要求碰撞双向反作用；第 5 章使用叉车与行人（`:107`）。
- **方案**：评审决定保留 kinematic test actor（确定性对抗），另设 dynamic physical actor；仅当第 5 章需要"撞击后反应"时升级。
- **验收**：评审后定义；当前仅登记。
- **范围与依赖**：SIM 物理；无依赖。

#### F3 视觉与碰撞一致性工具（审计 P1-9）[P1，部分完成]

- **依据**：审计 P1-9（`docs/audit-algorithm-sim-best-practice-2026-10-25.md:171`）要求 viewer 增加 collision geometry、COM、contact、axis overlay，并建立静置接地、轮心高度、关节轴、悬空部件、mesh-to-collision 偏差和 timestep sensitivity 测试。`8fa94cb` 已提交碰撞几何线框：`MujocoBackend::read_collision_geoms` 经 `World::collision_geoms`、`PresentationSnapshot.collision_geoms`、`FrameBody.collision_geoms` 接到 viewer 的 `draw_collision_geom_wireframe`（`viewer_main.cpp:2958`、`:4040`）；只覆盖活动 link，静态世界几何由地图自身渲染、不重复绘制（`docs/release-continuation-plan.md:252-254`）；新增 `--physics-overlay`（`viewer_main.cpp:4306`、`:5040`），与 K 键共用 `sim_truth_debug_allowed` 门禁，experience 运行期拒绝（`docs/release-continuation-plan.md:258-259`）；`tests/test_mujoco_backend.cpp` 断言导出非空且位姿有限（`docs/release-continuation-plan.md:259-260`）。本地 `--physics-overlay` 截图中，叉车周围可见洋红线框（姿态错误，见 F3a），导流柱、信号塔等 `surface_only` 道具没有线框。测试侧：`tests/` 中没有以 mesh-to-collision 偏差或轮心高度为名的测试（关键词检索无相关命中），其余各类的覆盖情况需要验证；场景清单现有的一致性校验见 A2。
- **方案**：在现有叠加层上补静态世界几何（world box）的线框，使绑定的 GLB 视觉与碰撞盒可同屏对照；P1-9 列出的测试按类逐项核对，缺失的补齐（偏差阈值由评审确定）。
- **验收**：overlay 能同时显示 actor proxy 与 world box，并与绑定的 GLB 视觉对照（A2 完成后三件立体道具也在其中）；P1-9 列出的六类测试各有对应用例或书面豁免；`ctest` 通过。
- **范围与依赖**：SIM 与 Viewer 的呈现层，不进入 ROBOT 观测；依赖 F3a（线框姿态）；排在 A2 之后，立体道具绑定碰撞盒后才有 world box 可对照。

#### F3a 碰撞几何线框的姿态与形状绘制错误（`8fa94cb` 引入）[P0]

- **依据**：
  - `draw_collision_geom_wireframe`（`viewer_main.cpp:2958-3015`）把中心点经 `to_raylib` 换到 raylib 基（`:2960`；`to_raylib(x, y, z)` 返回 `{x, z, -y}`，见 `:2700-2703`，相机 up 为 `{0, 1, 0}`，见 `:2713`），朝向向量却由 SIM 四元数旋转 SIM 基向量得到（`local_z` 在 `:2961-2965`，`local_x`、`local_y` 在 `:2973-2976`），没有换基就与中心点相加（box 角点 `:2983-2986`，圆柱与胶囊端点 `:2997-2999`）。同一文件中的接触法向做了逐分量换基 `{normal_x, normal_z, -normal_y}`（`:4032-4035`）。
  - 后果：SIM 中直立的 actor 代理圆柱（`mujoco_build.cpp:1207-1210`：半径 `max(0.01, radius_m)`，`pos[2] = max(0.05, height_m * 0.5)`）在 viewer 中的轴向落在 raylib 的 z 方向（SIM 的 -y），线框横躺着穿过直立的实体。本地截图的 3 倍放大图可见（`build/_shots/18_crop_left_forklift.png`、`19_crop_right_forklift.png`）。
  - box 代理在代码中只在 8 个角点各画一个边长 0.006 m 的线框小方块（`:2987`），没有边线。机器人底盘含 box 几何（`body_shell`、`ballast`，`mujoco_build.cpp:705`、`:709`），本地截图 `16_robot_overlay_front.png`、`17_robot_overlay_side.png` 中机器人的代理几乎不可见；截图中各代理的形状类别未逐个核对，需要验证。
  - 测试只断言导出非空、`valid` 与 `x_m`、`y_m`、`z_m`、`qw` 有限（`tests/test_mujoco_backend.cpp:129-140`），不检查朝向与形状，所以该错误没有被发现。
  - 已核对无误的部分：矩阵到四元数的转换；`read_back()` 的调用点（`mujoco_step.cpp:745`、`mujoco_build.cpp:1446`）；仅在 `retain_episode_history_` 时捕获（`scenario_session.cpp:296`）；ROBOT 边界（`git grep collision_geom -- src/robot` 无命中，`tests/check_boundary.py` 输出 `robot boundary ok`）。
- **方案**：线框展开按 `to_raylib` 同一映射换基；box 改为绘制 12 条边；把“换基并展开为线段与端点”抽成不依赖 raylib 的纯函数（输入 `FrameGeom`，输出线段与端点），放入独立 TU，沿用 `src/sim/viewer` 中 `frame_builder`、`particle_system`、`physical_audio` 的拆分方式。
- **验收**：（1）先写失败的单测：直立圆柱的两端点在 raylib 的 y 方向相距 `2 * half`；box 展开为 12 条边，边长等于各轴的 `2 * half`；（2）`--physics-overlay` 下 research_base 的叉车线框与直立实体对齐，前后对比截图存入 `build/_shots/`；（3）`test_mujoco_backend`、`robot_boundary` 与 `truth_isolation` 通过。
- **范围与依赖**：仅 Viewer 呈现层，不进入 ROBOT 观测；无依赖；A1 与 F3 余项的对比截图排在其后（4.2）。

#### F4 默认传感器 profile（审计 P1-7）[P2]

- **依据**：`configs/sim/default.json` 中 `imu_realism.gyro_noise_radps`、`imu_realism.gyro_bias_stddev_radps`、`imu_realism.gyro_bias_walk_radps_sqrt_s`、`imu_realism.accel_noise_mps2` 均为 0，`delivery.latency_steps` 为 0，`lidar.beams` 为 72、`lidar.max_range_m` 为 10.0；审计 P1-7 要求 `ideal`、`nominal`、`adverse` 三类显式 profile。第 1 章"传感器不是上帝视角"（`sim-game-design.md:61`）的教学效果依赖可见的噪声与延迟。
- **方案**：按审计 P1-7 建立三类 profile，并校验 `format`、未知字段与物理范围；profile 选择可作为 E1 与 F1 的变量来源。
- **验收**：三类 profile 有校验与负例测试；发布测试使用 nominal，故障测试使用 adverse（审计 P1-7 原文）。
- **范围与依赖**：`SimConfig`；与 E1、F1 相关。

#### F5 native runtime 仍使用占位适配器 [P3]

- **依据**：`src/robot/app/main.cpp:22` 的 `SafeHoldAdapter` 与 `:98` 的实例化；`src/robot/app/native_runtime.hpp:25` 的 `INativeAdapter` 与 `:51-56` 的 `NativeRuntime`（决策线程与 watchdog 线程）已存在，审计 P0-4 的"一次 tick 后退出的 bringup stub"描述与现状不符；设计文档第 9 章要求同一 ROBOT 配置从 SIM replay、HIL loopback 切到 native provider。
- **方案**：第 9 章立项前补真实 adapter；当前仅登记。
- **验收**：评审后定义。
- **范围与依赖**：ROBOT 应用层；无依赖。

#### F6 机械臂 [P3，后置]

- **依据**：战役章节表（`sim-game-design.md:53-146`）没有任何一章使用机械臂；机械臂只出现在 G6（`:411-415`）与"未来/后续"措辞中（`:25`、`:225`、`:248`、`:271`）；当前实现为契约（`arm_count`、`arm_rad[4]`）加冒烟测试（`tests/test_mujoco_backend.cpp:234-246`）。
- **方案**：机械臂控制与验收后置到有对应章节之后。
- **验收**：无（后置）。
- **范围与依赖**：无。

---

## 4. 依赖、顺序与实施计划

### 4.1 依赖与批次

- 第 1 批（P0，小而确定）：A1、A2、F3a。
- 第 2 批（P1，可并行）：
  - 可读性线：B2，随后 B3、B4；B1；B5。
  - 内容线：F1，随后 C1、C2。
  - 场景线：D1（依赖 A2）、D4；F3（部分完成，余下部分依赖 F3a，排在 A2 之后）。
- 第 3 批（P2）：C3、C4、D2、D3、E1、E2、B6、F4。
- 后置（P3）：F2、F5、F6。
- 依赖关系汇总：F1 → C1、C4、E1；A2 → D1；B2 → B3、B4；F3a → F3 余项；排期上 F3a → A1，A2 → F3 余项。

### 4.2 两条工作线

划分依据是文件冲突。`viewer_main.cpp` 有 5707 行，是 `src` 中最大的源文件（第二大为 `src/robot/autonomy/mapping/slam.cpp`，1705 行；`src/sim/viewer` 内第二大为 `frame_builder.cpp`，803 行）。依据中引用其行号的条目共 10 项（A1、B2、B3、B4、B5、B6、C3、D2、F3、F3a；A2 与 E1 在正文提到该文件，A2 不修改它，E1 属于评审后再定），B1 的相机与 3D 绘制入口也在该文件（`camera_from_state` 在 `:2705`，`draw_frame_3d` 在 `:3018`）。这些条目同时修改会产生冲突，因此串行；其余条目互不占用该文件，并行。A2 的规则落在 `scene_manifest.cpp`，内容在 `configs/`，不修改 `viewer_main.cpp`，属于线 D。

| 线 | 条目与顺序 | 说明 |
|---|---|---|
| 线 V（Viewer，串行） | F3a → A1 → B2 → B4、B5 → B1 → F3 余项 → B3、D2、B6、C3 | 同一时间只有一个条目修改 `viewer_main.cpp` |
| 线 D（非 Viewer，并行） | A2 → D1；D4；F1 → C1、C2；D3；F4 | 各分支之间互不占用 `viewer_main.cpp` |

线 V 的顺序理由：

- F3a 先于 A1：A1 的前后对比截图要在 `--physics-overlay` 下确认碰撞代理与模块化零件对齐，线框姿态正确后截图才能作为验收证据。
- B2 先于 B3、B4、B5：B3、B4 已声明依赖 B2 的缩放系数；B5 的证据文字对比度阈值与 B4 共用。
- B1 在 B2 之后：B1 要在三种窗口尺寸下验证，并计入 Turbo 的 12/24 ms 帧预算（`docs/sim-game-design.md:378`）。
- F3 余项排在 A2 之后：A2 让三件立体道具绑定碰撞盒，overlay 才有对应的 world box 可与视觉对照。
- B3、D2、B6、C3 互不依赖、改动面小，放在线 V 末尾。
- 每个条目把可单测的判定逻辑抽成独立 TU，缩短对 `viewer_main.cpp` 的占用；`src/sim/viewer` 已有 `frame_builder`、`particle_system`、`physical_audio` 等先例。

线 D 的顺序理由：

- A2 → D1：D1 依赖 A2（D1 条目已声明）。
- F1 → C1、C2：C1 依赖 F1（C1 条目已声明）；C1 与 C2 都修改 `configs/experiences/` 下的 manifest，同线顺序做以避免冲突；F1 动工前先确认 `EpisodePair` 的 digest 是否包含注册表内容（第 6 节）。
- D4 需要决策 3 的来源事实；D3、F4 独立，可穿插。

### 4.3 验收门禁

| 门禁 | 前置条目 | 通过条件 | 依据 |
|---|---|---|---|
| G1 视觉验收 | F3a、A1、A2、F3 余项、B1、B2、B4、B5、D4 | （1）Release 构建下，1280×720、1920×1080、2560×1080 三种尺寸的首章英雄镜头、简报、运行 HUD、复盘截图无关键遮挡、无裁切；（2）overlay 下碰撞线框与实体对齐，A2 的三件立体道具与其碰撞盒对齐；（3）`assets/models/manifest.json` 中 `NOASSERTION` 为 0；（4）Release 重测的 Viewer Turbo 倍率、`signal_town` 完整任务的 RTF 与 ROBOT tick p99 不退化，容差由决策 6 确定 | `docs/sim-game-design.md:537`（720p、1080p、超宽屏无关键遮挡）、`:363`（来源债务清零）；`docs/release-continuation-plan.md:248`（Release 视觉与碰撞代理一致性验收）；基线 `docs/release-performance-2026-10-27.md:21`（Viewer Turbo 6.35×、317 steps/s）与 `:35-36`（5901 步、RTF 6.34×、ROBOT tick p99 6.432 ms） |
| G2 内容验收 | F1、C1、C2 | 两章通过 manifest 校验与 `EpisodePair` 校验，四项裁决可见，每章闭环在 60–90 秒内 | `docs/sim-game-design.md:395`（四项裁决）、`:423`（60–90 秒闭环） |

评审后再定：E1、E2、C4（E1 与 C4 是否进入战役见决策 7）。后置：F2、F5、F6。

### 4.4 待定决策与输入

| 编号 | 待定事项 | 提供者 | 阻塞范围 |
|---|---|---|---|
| 决策 1 | A2 的规则 R1 放在哪里，以及是否同时做 R2（见下方决策记录） | 用户 | A2 |
| 决策 2 | 三个立体件的内容处理：绑定碰撞盒并移位，或其他方式 | 用户（策划决定） | A2 的提交 |
| 决策 3 | 39 项 `NOASSERTION` 资产的真实来源与许可证 | 资产作者 | D4；发布资产包（`sim-game-design.md:363`） |
| 决策 4 | UI 缩放系数的算法（按窗口高度或 DPI）与最小字号 | 评审 | B2；B3、B4、B5 间接依赖 |
| 决策 5 | 可见像素占比与对比度阈值（候选：WCAG AA 的 4.5:1） | 评审 | B1、B4、B5 的验收 |
| 决策 6 | G1 的性能容差，以及基线的 run-to-run 波动范围 | 评审与实测 | G1 |
| 决策 7 | E1、C4 是否进入战役 | 评审 | E1、C4 |

#### 决策记录 1：A2 规则 R1 的位置

**背景**：A2 要拦住的缺陷是“实体立体件以 `surface_only` 通过校验，且没有碰撞体”。设计文档 `:364` 已规定契约，加载器只实现了前半句（`scene_manifest.cpp:168-169`）。最初的设想是增加 footprint 与 place、actor 路径的相交规则，需要把 places 与 actors 传入加载器；核对后发现：（1）不限高度时 10 个命中中有 7 个是设计意图的贴地件；（2）立体件绑定碰撞盒后，叉车路径由现有 actor 路径校验覆盖；（3）R1 只需要实例自身与资产清单，`parse_instance` 已持有这两项。

**选项**：

- 选项 1：R1 放入 `parse_instance`（`scene_manifest.cpp:167-173`，紧邻现有离地校验），加载期拒绝；`test_scene_manifest.cpp` 增加负例；不改签名、不新增输入、不修改 `viewer_main.cpp`。与设计文档 `:364` 的“会拒绝加载”及现有绑定校验同一风格。风险：规则与内容必须在同一提交内落地，否则 viewer 无法加载 research_base（`viewer_main.cpp:4516-4524` 在场景加载失败时以非零状态退出，`:4824-4830` 拒绝切换 Experience）。
- 选项 2：R1 只在测试里断言（按目录枚举所有 `*.scene.json` 检查高度），加载器不变。优点：表现层违规不会阻断 viewer；缺点：违规场景在运行期仍可加载，保护依赖测试被执行，与 `:364` 的“拒绝加载”不一致。
- 选项 3：footprint 与 place、actor 路径的相交校验。缺点：需要新模块，并把 places 与 actors 传入加载器（`load_scene_manifest` 当前输入只有资产清单与静态 box，见 `scene_manifest.hpp` 的声明）；不限高度产生 7 个误报；限定高度后它只作用于立体的 `surface_only` 实例，而这类实例在选项 1 下不允许存在。

**建议**：选项 1；R2 作为可选加固另行安排。理由：直接落实已写入设计文档的契约；单实例、无新输入；与现有校验同层；不占用线 V；在现有数据上恰好拒绝 3 个目标实例、零误报。

**后果**：A2 的提交范围为 `scene_manifest.cpp`、`tests/test_scene_manifest.cpp`、`configs/worlds/research_base.json` 与 `configs/worlds/research_base.scene.json`（内容处理按决策 2）；research_base 任务或实验测试因新增碰撞盒而受影响时，在同一提交内调整并说明。

**状态**：待确认（0.2 编写时尚未决定）。

### 4.5 完成标准

- 每个条目一个独立提交（`docs/release-continuation-plan.md:13`、`:163`），不混入无关的未提交文件（`:185`）。
- 缺陷类条目（A1、A2、F3a）先写失败的测试，再修复。
- 每个条目：相关测试、`robot_boundary`（`CMakeLists.txt:897`）与 `truth_isolation`（`CMakeLists.txt:386`）通过。
- 视觉类条目：Xvfb 前后对比截图存入 `build/_shots/`（被 git 忽略，复现方法见附录 B）。
- 提交后更新 `docs/release-continuation-plan.md` 的提交号、测试结果与剩余风险（`:171`）。
- 每道门禁前跑全量 Debug CTest（`CMakeLists.txt` 中有 78 处 `add_test(`）；影响渲染或仿真的条目在门禁时做 Release 重测。

### 4.6 规模估算

以下是估算，不是证据。划分依据是条目的拆分粒度（每个子任务独立提交，`docs/release-continuation-plan.md:163`），不是历史工时。

| 规模 | 单项提交数 | 条目 | 小计 |
|---|---|---|---|
| S | 约 1 | F3a、A1、B6、C3 | 4 |
| M | 约 2–3 | A2、D4、F3 余项、B3、B4、B5、C2、D2、D3、F4 | 20–30 |
| L | 约 4–6 | B2、B1、F1、C1、D1 | 20–30 |

合计约 44–64 个提交，不含评审后再定（E1、E2、C4）与后置（F2、F5、F6）。参照节奏：`git log` 统计 2026-10-24 至 10-27 每日提交数为 42、43、124、26（10-27 为核对时的当日累计）。提交数不是瓶颈；瓶颈是 4.4 的待定决策与 4.3 的两道门禁。

## 5. 风险与回退

- 窗口缩放与 UI 缩放改动面大（B2、B3、B4）：分两步，先缩放系数后布局；回退为默认 1100×750 与系数 1.0。
- 相机遮挡淡出（B1）影响帧预算与截图基线：Turbo 下降级为不淡出（沿用 `sim-game-design.md:378` 的预算规则）；回退为关闭开关。
- 装配玩法（E1）触及 ROBOT 配置域：受 `tests/check_boundary.py` 保护；回退为不启用零件选择，保持 `standard` 零件。
- 进度存档（E2）文件损坏：回退为空 profile 并提示，不阻塞游玩。
- A2 的规则 R1 会让 research_base 的 3 个立体件加载失败，贴地件不受影响：规则与内容必须在同一提交内落地；新增碰撞盒会改变 research_base 的可通行空间，对任务与实验测试的影响需要验证；回退为还原该提交。
- 线 V 串行会拉长 Viewer 条目的整体排期：缓解方式是每个条目把可单测的判定逻辑抽成独立 TU，缩短对 `viewer_main.cpp` 的占用。
- G1 的 Release 重测基线测于第 4 阶段（D1 已记录该前提需要验证）；基线的波动范围未知时无法设定容差（决策 6）。
- 变量注册表改为表驱动（F1）对 `EpisodePair` digest 的影响需要验证：先确认 digest 是否包含注册表内容；迁移后以首章现有测试做回归。

## 6. 待验证与开放问题

1. A2：`surface_only` 顶面高度如何由 `position_m[2]`、`scale[2]` 与资产的 `expected_bounds_m`、`pivot_m` 计算；做 R2 时到达半径取 SIM 侧 0.25 m 还是 ROBOT 侧 0.30 m。
2. A1：actor 绘制分支抽取为可单测函数的改动量。
3. B3：当前 raylib 版本对 SDF 字体的支持，以及 CJK 预加载策略的兼容性。
4. B1、B4：可见像素占比与对比度阈值由评审确定。
5. C1、F1：候选变量（安全章速度上限、SLAM 章里程计打滑或噪声强度）对应的配置项是否存在，是否适合作为 typed 变量。
6. C2：新人试玩的首选项命中率。
7. C3：转场时镜头先于字幕切换是否为有意的镜头提前量。
8. B6：标签规则（保留英文工程标签，或全部中文化）。
9. D1：通用件是否拆出 `environment.research_base.*` 命名空间；`release-performance-2026-10-27.md` 的基线需要重测。
10. F1：注册表改动对 `EpisodePair` digest 的影响。
11. E1：装配玩法是否进入战役。
12. F3：审计 P1-9 列出的静置接地、关节轴、悬空部件、timestep sensitivity 各类测试的现有覆盖情况；本次只确认 `tests/` 中没有以 mesh-to-collision 偏差或轮心高度为名的测试。
13. A2：R1 的位置与三个立体件的内容处理已列为决策 1、决策 2（4.4）；`safety_lab.scene.json` 是否被 `test_scene_manifest` 之外的测试加载，需要验证；设计文档 `:306` 的“目标 footprint 可达”在代码中的实现位置，需要验证。
14. A2 的 R2：`wander`、`constant-velocity` 类 actor 没有固定路径（`src/sim/world/actor_behavior.cpp:48-51` 注册了三种行为），路径类校验的取法需要确定；现有 14 个 actor 全部为 `waypoints`。
15. G1：Release 重测的容差与基线的 run-to-run 波动范围（决策 6）。

---

## 附录

### A. 相关文档

- `docs/sim-game-design.md`：产品与游戏设计，章节、环境 kit、验收条件的出处。
- `docs/release-continuation-plan.md`：阶段计划与续作入口。
- `docs/audit-algorithm-sim-best-practice-2026-10-25.md`：ROBOT 算法与 SIM 真实性审计（P1-7、P1-8、P1-9 的出处）。
- `docs/release-performance-2026-10-27.md`：Release 倍率与确定性验收基线。

### B. 证据复现

- 构建：`cmake --build build --target sim_viewer`。
- 研究基地近景截图：`xvfb-run -a -s "-screen 0 1280x800x24" ./build/sim_viewer --sandbox --3d --map configs/worlds/research_base.json --mute --locale zh-CN --camera-target 18.0 6.85 --camera-dist 7 --camera-pitch 28 --camera-yaw 0 --shot 60 NAME.png`。raylib 把 PNG 写到当前目录（只用文件名），需自行移走，避免进入仓库。
- 碰撞代理线框截图：在上一条命令中加 `--physics-overlay`（`8fa94cb` 新增，沙盒模式可用，experience 运行期拒绝）。
- 首章截图：`--play --locale zh-CN --mute --shot N NAME.png`，N 取 15、40、120、400 分别对应开场镜头与简报。
- A2 数值核对：取 `assets/models/manifest.json` 中资产的 `expected_bounds_m`，按 `*.scene.json` 实例的 `yaw_deg` 与 `position_m` 变换为轴对齐矩形；与 `*.actors.json` 中每台 actor 的 waypoints 线段取最小距离并与 `radius_m` 比较；与 `research_base.json` 的 `places` 坐标比较。
- A2 高度分类与对比方案：在上一条的基础上，取实例资产的垂直范围（`expected_bounds_m` 的 z 范围乘 `scale[2]`），只统计 `surface_only` 实例，以 0.10 m 分为贴地件与立体件；actor 路径取 `x`、`y` 为起点，加 `waypoints`，`loop` 为真且多于一个 waypoint 时回到首个 waypoint（与 `actor_sidecar.cpp:54-59` 一致）；place 取世界内全部 place，到达半径取 0.25 m 与 0.30 m 两档；命中条件为 footprint 到 place 的距离小于半径，或 footprint 到路径的距离小于 actor 的 `radius_m`。
- place 与静态 box 的净空：对 `configs/worlds/*.json`（排除 `.scene.json` 与 `.actors.json`）中每个 place，取它到每个静态 box 的 `x0`、`y0`、`x1`、`y1` 矩形的最小距离，统计小于 0.30 m 的数量（结果为 0）。
- F3a 复现：在上述截图命令中加 `--physics-overlay` 并取叉车近景，把叉车区域放大 3 倍核对线框轴向。
- Viewer 条目统计：按 `#### ` 标题切分本文档，统计依据中带 `viewer_main.cpp:行号` 的条目；A2 与 E1 只在正文提到该文件，不计入；文件行数用 `wc -l`。
- 提交节奏：`git log --format=%cd --date=short | sort | uniq -c`。
- 字号统计：对 `viewer_main.cpp` 中的 `draw_ui_text(` 调用按括号配对解析第 4 个实参，统计字面量字号。
- 贴图与音频引用统计：`git grep -F <文件名> 8fa94cb -- src configs tests`。

### C. 修订历史

| 版本 | 日期 | 修订人 | 修订描述 |
|-----|------|-------|---------|
| 0.2 | 2026-10-27 | Cursor Agent | 新增 F3a（`8fa94cb` 碰撞线框姿态与形状错误，P0）；A2 按设计文档 `:364` 的契约重写为“`surface_only` 必须贴地”，并记录高度分类、误报对比、现有保护与缺口；F3 去掉与 A2 共用 footprint 的依赖；第 4 节扩展为实施计划（两条工作线、两道验收门禁、七项待定决策、完成标准、规模估算）；条目 24 项增至 25 项，P0 由 2 项增至 3 项 |
| 0.1 | 2026-10-27 | Cursor Agent | 初始版本：基于 `e6e17fd` 的证据核对，在 `8fa94cb` 上复核行号与 F3 状态，登记 24 项 backlog |
