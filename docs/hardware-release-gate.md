# Real hardware release gate

软件仿真和 native HIL 契约已经通过；完整 1.0 仍需要目标板卡的现场证据。记录模板是 [hardware-release-gate.template.json](hardware-release-gate.template.json)，校验器是 [`tools/validate_hardware_release_record.py`](../tools/validate_hardware_release_record.py)。

```sh
python3 tools/validate_hardware_release_record.py path/to/hardware-release-gate.json
```

校验器会 fail closed：模板和 nominal calibration 只能得到 `PENDING_EXTERNAL` 或失败；只有 measured calibration、正数实测指标、八项 gate 的证据引用和没有 pending/failed gate 时，才允许 `overall_status=PASS`。

现场记录必须同时保存：

1. board/profile/firmware identity、robot description hash 和 calibration artifact SHA-256；
2. MCU control/feedback period、transport latency p95/max、单次与连续丢包结果；
3. 急停输入到机械停车的距离和时间；空载、满载、低抓地制动距离；
4. 实际 LiDAR 量程/profile、温度与负载范围；
5. 与 native HIL 相同的固定回放日志、现场长时间回放和 operator/date/evidence path。

任何一项缺少原始记录、只使用 nominal profile、或由 SIM truth 推导，都不能填写 `PASS`。现场证据完成后，把 JSON、原始日志和 SHA-256 清单一起归档，再重新执行 Debug/Release 全套软件门禁。
