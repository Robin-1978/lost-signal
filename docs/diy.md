# DIY 作者手册：玩家自造硬件与内容

原则：硬件是数据，算法是代码。`test_player_pack` 把这条边界钉死了——
第三方轮胎纯 JSON 就能上车完赛；新算法/新底盘类型要 C++，但缝已经开好。

## 纯 JSON 可加（无需改代码，门禁全过即合法）

| 加什么 | 文件 | 约束 |
|---|---|---|
| 新轮胎/电机/雷达/IMU/算力 | `assets/components/*.json`（新文件，slot 对应） | behavior 数字表齐（门禁按槽位检查键）；cost 进预算 |
| 整车（已有底盘类型） | `assets/robots/*.json` | `drive.type` 三选一；增益全部自整定，手调 Kp 直接出局 |
| 新底盘底盘参数 | 同上 | 同上 |
| 关卡 | 管线 spec（`tools/generate_levels.py` LEVELS 加条目）或手写 JSON（过 schema＋validator） | 对比机制（动态/打滑/长度三选一），否则策展否决 |
| 地图 actors/markers | `assets/maps/*.json`（add-only＋`active_in` 隔离） | round-trip 门（字母序 markers、canonical 格式） |
| 能力/奖励 | `assets/abilities.json` | `ability.*` 前缀，大声 resolve |
| 车库底盘选项 | presentation `chassis_options`（opt-in，缺席＝固定） | robot 须存在；unlock 按 `unlock_ability` |

## 必须 C++（缝已开好，按菜谱来）

新 `drive.type`（如 swerve）：实现 `ChassisModel`（`step`/`telemetry`/`caps`，
遥测填统一 `ChassisTelemetry` 结构）＋ `ChassisBuildParams` 组装 creator
（放自己 `.cpp`，仿三份现成 creator）＋ `register_builtin_chassis` 加一行
＋ `CMakeLists.txt` 加文件＋ robot JSON 写新 type。`mission`／工厂零改动。
静态自注册已否决（archive 链接器丢弃未引用目标文件，实测翻车）——显式一行，
不要魔法。

新算法实现（如第二个 Predictor）：接口在第二个实现出现时才开（现行冻结规则），
在此之前单实现直接写纯函数＋差分测试。

## 玩家包工作流（命令）

```bash
cp -r assets /tmp/mypack            # 永远 overlay，不改 shipped 文件
# ... 加你的 JSON ...
python3 tools/validate_content.py   # 门禁（schema＋unlock 链＋预算）
python3 tools/generate_levels.py --check   # 管线无漂移
python3 tests/test_content_roundtrip.py -q # canonical 格式
./tools/build.sh --headless --test  # 全回归（你的包不能翻 pinned 表）
```

## 保证与红线

- 确定性：同内容＋同配置＋同种子逐比特一致；翻了就是 bug，不是噪声。
- 诚实物理：按文献/手册填数（见 `sim2real.md` §2）；为过关调数＝调参作弊，
  门禁直接否决（voucher：Family C signal_pocket否决案）。
- UI 分支：新遥测形状要在 renderer 加分支（桌面目视验收，headless 断言不到，
  先声明再合并）。
