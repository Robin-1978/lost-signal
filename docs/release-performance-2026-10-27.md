# Release 倍率与确定性验收（2026-10-27）

本报告记录 `refactor/clean-rebuild` 在 Release 构建下的固定步仿真验收。所有模式都执行 20 ms scenario transition 和其中相同的 1 ms MuJoCo 子步；Viewer 倍率只控制每个墙钟帧执行多少个 transition。

## 构建与环境

- 构建：`cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DLOST_SIGNAL_WITH_VIEWER=ON`
- Viewer：Xvfb + Mesa llvmpipe，`FAST` 画质，1280×1024 虚拟显示。Viewer 数据反映本机软件渲染吞吐，不是 GPU 产品基准。
- ROBOT：`configs/robot/differential.json`
- SIM：`configs/sim/default.json`
- 性能场景：`configs/worlds/industrial_200k.json`，固定 240 个显示帧；任务在测量窗内不会提前结束。
- 发布任务：`configs/worlds/signal_town.json`，五个公开 place ID，8000 步预算和 `--require-pass`。

## Viewer 实测

| 请求模式 | 实际 RTF | steps/s | tick p99 | tick max | 结果 |
|---|---:|---:|---:|---:|---|
| 1× | 1.00× | 50 | 15.43 ms | 28.55 ms | 达到请求倍率 |
| 8× | 5.47× | 273 | 6.79 ms | 7.78 ms | 显式受本机吞吐限制 |
| 64× | 5.38× | 269 | 6.81 ms | 8.46 ms | 显式受本机吞吐限制 |
| Turbo | 6.35× | 317 | 6.66 ms | 9.33 ms | Viewer 最大吞吐 |

8×/64× 的请求高于本机软件渲染与仿真组合吞吐。调度器保留固定物理步长并通过实际 RTF/LIMIT 暴露限流，不丢传感器周期，也不放大 `dt`。

## Headless 实测

同一 20 万平方米场景执行 1200 步（24.00 s sim time）：

- 墙钟 3.747 s，RTF 6.40×，320.2 steps/s。
- ROBOT tick p50/p95/p99/max：1.278/5.251/9.249/18.929 ms。
- 0 碰撞；1200/1200 observations accepted；最大定位误差 0.159 m。

发布配置 `signal_town` 完整任务：

- 5901 步，118.02 s sim time，墙钟 18.611 s，RTF 6.34×，317.1 steps/s。
- ROBOT tick p50/p95/p99/max：2.173/4.770/6.432/14.146 ms。
- 5/5 objectives，全部 ROBOT done 且通过独立 SIM arrival；0 碰撞。
- path length 100.056 m；最大定位误差 0.278 m；最小平面 clearance 0.191 m。

## 结果一致性门禁

`test_playback_controller` 对 1×、8×、64×、Turbo 和直接 headless 执行逐 transition 精确比较：

- scan/motion/IMU acquisition stamp、observation sequence 与接受状态；
- 完整 motion command 与有效期；
- 静态/动态碰撞；
- 最终 6DoF body pose、平面速度与 yaw rate；
- 未完成运动任务和已稳定完成任务的 ROBOT/SIM terminal verdict；
- 完整 `EpisodeScore`，包括观测、碰撞、路径、定位误差、clearance、terrain 与 energy 字段。

相关 Debug 与 Release 测试均通过。倍率没有进入 ROBOT 输入，也没有改变仿真结果。

## 复现命令

```sh
./build-release/test_playback_controller

./build-release/lost_signal_sim \
 --robot configs/robot/differential.json \
 --sim configs/sim/default.json \
 --map configs/worlds/signal_town.json \
 --objectives energy_hub,freight_yard,housing_door,market_gate,town_gate \
 --steps 8000 --profile --require-pass --out signal_town-release.json

xvfb-run -a ./build-release/sim_viewer --sandbox \
 --robot configs/robot/differential.json \
 --sim configs/sim/default.json \
 --map configs/worlds/industrial_200k.json \
 --rate 8 --quality fast --mute --shot 240 viewer-rate8.png
```
