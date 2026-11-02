# First Contact 内容清单

本清单用于发布前逐项核对双语资源。运行时文案的唯一来源是两个 manifest 文件。

| ID | English source | zh-CN source | Gate |
|---|---|---|---|
| `intro` | Four 6.0 s hero shots: drive wheel, caster, LiDAR, mission | 同一顺序、同一时长和同一组件 ID | 可跳过，不能推进 sim time |
| `walkaround` | Four component cards; all optional in First Contact | 四张部件卡；首章均为可选检查 | 3D hotspot/accessibility action only |
| `scan_cadence` | `lidar.scan_period_s`, 0.02 s baseline / 0.10 s variant | 同一 typed variable 与数值 | Only one controlled variable |
| `prediction` | Three evidence rules: safe non-equivalent difference / equivalent / unsafe | 三个互斥的实测证据规则，顺序一致 | Direction-neutral timing or clearance change; no authored `correct` flag |
| `debrief` | ROBOT, SIM safety, mission and experiment verdicts | 机器人、仿真安全、任务和实验 verdict | Evidence missing stays insufficient |
| `lessons` | Measurements, independent arrival, collision hard-fail | 测量、独立到达、碰撞硬失败 | Publish only after gate passes |

检查命令：

```sh
./build/test_experience_manifest
./build/test_experience_flow
./build/test_experience_mission
./build/test_ab_experiment
```
