#!/usr/bin/env python3
"""关卡生产管线 v1：模板 + 参数表 -> 关卡/表现/剧情/章节装配。

模板只表达“已验证题型的参数化”（本期仅 combo 回响型），不发明新机制；
输出文件确定、可 diff；`--check` 用于 CI 门禁。
地图等共享内容是生成器的输入断言，不是输出（见 REQUIRE_MAP）。
"""
from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

REQUIRE_MAP = {
    "research_base": {
        "markers": ["robot_bay", "supply_station", "signal_tower"],
        "actors": ["forklift_a"],
    },
    "signal_town": {
        "markers": ["town_gate", "energy_hub", "market_gate"],
        "actors": ["town_cart_a", "town_cart_b"],
    },
}

LEVELS: list[dict] = [
    {
        "id": "signal_echo",
        "template": "echo",
        "order": 9,
        "chapter": 2,
        "title": "信号回响",
        "map": "research_base",
        "prerequisites": ["lost_signal"],
        "seed": 48,
        "time_limit_s": 150,
        "budget": 7,
        "arrive_tolerance_m": 0.62,
        "via_radius_m": 1.25,
        "safe_min_clearance_m": 0.3,
        "fast_time_s": 75.0,
        "efficient_energy_wh": 2.0,
        "concept": "dynamic_obstacle_prediction",
        "prediction_prompt": "同一条叉车通道再走一次，哪里最值得省时间？",
        "knowledge_card": "同样的路，换一套动力与感知组合，结果完全不同：先看证据，再谈速度。",
        "garage_heading": "换个跑法，再走一次",
        "garage_brief": "同一条叉车通道，预算更紧。洛一已经证明过自己，这次试试不同的动力组合。",
        "prediction_heading": "出发前，先做一个判断",
        "prediction_hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
        "prediction_choices": [
            {"id": "signal_echo.power_first",
             "text": "动力越强通过越快",
             "feedback": "动力要和算力匹配：扭矩电机配慢算力反而更容易撞，证据在等待事件里。"},
            {"id": "signal_echo.efficiency",
             "text": "省电的组合也能准时到",
             "feedback": "高效电机加节能胎确实省电，看看它能不能同时保住安全分。"},
            {"id": "signal_echo.static_map",
             "text": "把叉车永久写进静态地图",
             "feedback": "叉车会移动，静态地图只说明固定障碍，不能表示它下一刻的位置。"},
        ],
        "running_heading": "回响通道",
        "running_normal": "洛一：导航链路稳定，正在接近信号塔。",
        "running_detected": "洛一：锁定叉车，正在预测它的运动轨迹。",
        "debrief_success_title": "任务完成：回响确认",
        "debrief_success_summary": "同一条路，不同的跑法，洛一同样送达。",
        "debrief_failure_summary": "选择右侧证据，现场会跳到对应时刻。",
        "debrief_evidence_heading": "因果证据",
        "debrief_evidence_caption": "这些内容由本次仿真的事件与状态生成。",
        "debrief_knowledge_title": "动力与感知组合",
        "debrief_failure_titles": {
            "collision-dynamic": "任务失败：洛一与叉车发生碰撞",
            "collision-static": "任务失败：洛一碰到了固定障碍",
            "timeout": "任务失败：没有在时限内送达",
        },
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "computer": "computer.light",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.standard_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "rewards": ["ability.combo_mastery"],
        "story_before": "信号塔启动之后，仓库深处传来了另一段回响。洛一：再走一次吧，这次我们换个跑法。",
        "story_after": "洛一：同样的叉车，同样的路——不一样的是我们自己。",
    },
    {
        "id": "signal_hold",
        "template": "echo",
        "order": 20,
        "chapter": 2,
        "title": "信号守望",
        "map": "research_base",
        "prerequisites": ["signal_echo"],
        "seed": 42,
        "time_limit_s": 150,
        "budget": 8,
        "arrive_tolerance_m": 0.62,
        "via_radius_m": 1.25,
        "safe_min_clearance_m": 0.3,
        "fast_time_s": 100.0,
        "efficient_energy_wh": 2.5,
        "concept": "waiting_as_maneuver",
        "prediction_prompt": "同一条叉车通道，这一班叉车会封住走廊好几秒：省时间的办法是等，还是蹭？",
        "knowledge_card": "等待是机动，不是发呆：走廊释放就出发。影子评估记下了每次等待的初值、末值和实际时长，对得上才算数。",
        "garage_heading": "等，也是一种跑法",
        "garage_brief": "同一条叉车通道，这一班叉车会封路数秒。算力拉满也撞不过去的时候，安全地等过去就是最快。",
        "prediction_heading": "出发前，先做一个判断",
        "prediction_hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
        "prediction_choices": [
            {"id": "signal_hold.wait_for_release",
             "text": "看预测：走廊释放就出发",
             "feedback": "对。等待是机动，不是发呆：预测、等待、汇入是一整段行为，影子评估会给出最早出发时刻。"},
            {"id": "signal_hold.wait_forever",
             "text": "看到叉车就一直等，等它彻底走远",
             "feedback": "过度等待：走廊一旦可安全通过就该出发，一直等到彻底清空反而更慢。"},
            {"id": "signal_hold.squeeze_through",
             "text": "不等，直接蹭过去",
             "feedback": "默认配置就是这么开的：7 秒长等换成贴身通过，最小间隙 0.15 米，安全分没了。"},
        ],
        "running_heading": "守望通道",
        "running_normal": "洛一：导航链路稳定，正在接近信号塔。",
        "running_detected": "洛一：锁定叉车，它封住了走廊，停车等待安全窗口。",
        "debrief_success_title": "任务完成：守望确认",
        "debrief_success_summary": "封路数秒，等过去了，洛一同样送达。",
        "debrief_failure_summary": "选择右侧证据，现场会跳到对应时刻。",
        "debrief_evidence_heading": "因果证据",
        "debrief_evidence_caption": "这些内容由本次仿真的事件与状态生成。",
        "debrief_knowledge_title": "等待即机动",
        "debrief_failure_titles": {
            "collision-dynamic": "任务失败：洛一与叉车发生碰撞",
            "collision-static": "任务失败：洛一碰到了固定障碍",
            "timeout": "任务失败：没有在时限内送达",
        },
        "default_build": {
            "tires": "tires.grip",
            "sensor": "sensor.standard_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "rewards": ["ability.signal_hold"],
        "story_before": "洛一：这一班叉车不一样，它会把路封上好几秒。我们不等赢，就只能等着被撞。",
        "story_after": "洛一：等过去了。走廊是叉车的，我们借几秒。",
    },
    {
        "id": "long_haul",
        "template": "reach",
        "order": 13,
        "chapter": 2,
        "title": "长途穿越",
        "map": "research_base",
        "prerequisites": ["loose_ground"],
        "seed": 127,
        "spawn": "signal_tower",
        "goal": "calibration_pad",
        "goal_label": "标定点",
        "goal_visual": "calibration_pad",
        "time_limit_s": 150,
        "budget": 5,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "trio",
        "arrive_tolerance_m": 0.62,
        "safe_min_clearance_m": 0.2,
        "max_localization_error_m": 0.3,
        "fast_time_s": 46.0,
        "efficient_energy_wh": 0.72,
        "concept": "long_route_pacing",
        "prediction_prompt": "横穿整个仓库，速度和电量怎么分配？",
        "knowledge_card": "长途看配速：匀速比重脚油门更早到，温柔驾驶赢的是全程。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.long_haul"],
        "garage_heading": "跑个长途",
        "garage_brief": "信号塔回标定点，横穿整个仓库。没有叉车，考配速和耐心。",
        "prediction_heading": "长途，短板在哪？",
        "prediction_hint": "先做预测。匀速和冲刺，哪个先到标定点？",
        "prediction_choices": [
            {"id": "long_haul.link.pace",
             "text": "配速会决定胜负",
             "feedback": "如果全程重脚油门，时间和电量一起丢。看用时和电表。"},
            {"id": "long_haul.link.corner",
             "text": "转弯会浪费时间",
             "feedback": "如果进弯太晚，只能绕大圈。看转弯处的轨迹。"},
        ],
        "running_heading": "把洛一开回标定点",
        "running_normal": "洛一：长途巡航中，配速稳定。",
        "running_detected": "洛一：前方状态已确认。",
        "debrief_success_title": "长途抵达",
        "debrief_success_summary": "横穿仓库：匀速就是最快，温柔就是节能。",
        "debrief_failure_summary": "看看是哪一项评分丢了，长途最怕忽快忽慢。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "配速",
        "debrief_failure_titles": {
            "collision-static": "运送失败：碰到了障碍",
            "collision-dynamic": "运送失败：与叉车发生碰撞",
            "timeout": "运送失败：没有在时限内送达",
        },
        "story_before": "洛一：从信号塔回标定点，最远的一段路，我们慢慢开。",
        "story_after": "洛一：到家了。长途的秘诀就是没有秘诀：匀速。",
    },


    {
        "id": "scrub_climb",
        "template": "reach",
        "order": 14,
        "chapter": 2,
        "title": "履带爬坡",
        "map": "research_base",
        "prerequisites": ["long_haul"],
        "seed": 163,
        "robot": "robot.ro3",
        "spawn": "workshop_exit",
        "goal": "supply_station",
        "goal_label": "能源模块",
        "goal_visual": "supply_station",
        "time_limit_s": 150,
        "budget": 5,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "trio",
        "arrive_tolerance_m": 0.62,
        "safe_min_clearance_m": 0.0,
        "fast_time_s": 40.0,
        "efficient_energy_wh": 0.5,
        "concept": "tracked_detour",
        "prediction_prompt": "履带走窄走廊，打滑和转向哪个先出问题？",
        "knowledge_card": "履带过窄道：转弯全靠搓，慢一点，里程计才不撒谎。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.scrub_climb"],
        "garage_heading": "履带再战",
        "garage_brief": "同一条北上路，换履带车再走一次。转弯即打滑，慢就是稳。",
        "prediction_heading": "履带北上，短板在哪？",
        "prediction_hint": "先做预测。轮式和履带，同一条走廊走出什么不同？",
        "prediction_choices": [
            {"id": "scrub_climb.link.slip",
             "text": "转弯会丢定位",
             "feedback": "如果进弯太急，履带空转，里程计 drift 会先超标。看复盘里的定位误差。"},
            {"id": "scrub_climb.link.corner",
             "text": "转弯会浪费时间",
             "feedback": "如果带着速度进弯，只能绕大圈。看出弯点的轨迹。"},
        ],
        "running_heading": "履带北上",
        "running_normal": "RO-3：双履带在线，正在搓着转向。",
        "running_detected": "RO-3：前方状态已确认。",
        "debrief_success_title": "履带登顶",
        "debrief_success_summary": "窄走廊履带通过：慢进弯，定位才保得住。",
        "debrief_failure_summary": "看看是哪一项评分丢了，履带最怕急转弯。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "滑移转向",
        "debrief_failure_titles": {
            "collision-static": "履带失败：碰到了障碍",
            "collision-dynamic": "履带失败：与叉车发生碰撞",
            "timeout": "履带失败：没有在时限内到达",
        },
        "story_before": "洛一：北上那条路，让 RO-3 也走一遍。",
        "story_after": "RO-3：到了。同一条路，不同的脚，感觉完全不同。",
    },
    {
        "id": "sand_track",
        "template": "reach",
        "order": 15,
        "chapter": 2,
        "title": "沙地履带",
        "map": "research_base",
        "prerequisites": ["scrub_climb"],
        "seed": 233,
        "robot": "robot.ro3",
        "spawn": "supply_station",
        "goal": "signal_tower",
        "goal_label": "信号塔",
        "goal_visual": "signal_tower",
        "time_limit_s": 150,
        "budget": 6,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "sand",
        "arrive_tolerance_m": 0.62,
        "safe_min_clearance_m": 0.15,
        "fast_time_s": 60.0,
        "efficient_energy_wh": 0.8,
        "concept": "sand_scrub",
        "prediction_prompt": "沙地加履带，打滑乘以打滑等于什么？",
        "knowledge_card": "复合打滑：沙地吃抓地，履带吃转向，两样叠在一起，温柔是唯一的解。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.sand",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.sand_track"],
        "garage_heading": "沙地履带",
        "garage_brief": "同一条沙地路，换履带车。沙吃抓地，履带吃转向，一起来。",
        "prediction_heading": "复合打滑，短板在哪？",
        "prediction_hint": "先做预测。两种打滑叠加，定位会去哪？",
        "prediction_choices": [
            {"id": "sand_track.link.compound",
             "text": "定位会彻底漂掉",
             "feedback": "如果又滑又搓，里程计最后差几米。看复盘里的定位误差。"},
            {"id": "sand_track.link.wait",
             "text": "慢就是快",
             "feedback": "如果温柔驾驶，时间反而花得少（不迷路）。看用时。"},
        ],
        "running_heading": "沙地履带中",
        "running_normal": "RO-3：沙地行驶中，履带正在打滑。",
        "running_detected": "RO-3：前方状态已确认。",
        "debrief_success_title": "复合通过",
        "debrief_success_summary": "沙加履带都过去了：慢，且温柔。",
        "debrief_failure_summary": "看看是哪一项评分丢了，复合打滑最怕急。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "复合打滑",
        "debrief_failure_titles": {
            "collision-static": "履带失败：碰到了障碍",
            "collision-dynamic": "履带失败：与叉车发生碰撞",
            "timeout": "履带失败：没有在时限内到达",
        },
        "story_before": "洛一：沙地那条路，让 RO-3 也走一遍。",
        "story_after": "RO-3：到了。沙和履带，我全都要慢。",
    },


    {
        "id": "sand_steer",
        "template": "reach",
        "order": 18,
        "chapter": 2,
        "title": "沙地转向",
        "map": "research_base",
        "prerequisites": ["sand_track"],
        "seed": 257,
        "robot": "robot.ro2",
        "spawn": "supply_station",
        "goal": "signal_tower",
        "goal_label": "信号塔",
        "goal_visual": "signal_tower",
        "time_limit_s": 150,
        "budget": 6,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "sand",
        "arrive_tolerance_m": 1.0,
        "safe_min_clearance_m": 0.2,
        "max_localization_error_m": 0.3,
        "fast_time_s": 38.0,
        "efficient_energy_wh": 0.65,
        "concept": "sand_steer",
        "prediction_prompt": "转向车进沙地，方向盘还管用吗？",
        "knowledge_card": "沙地加转向：轮胎先打滑，方向盘后到，两个都要提前量。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.sand",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.sand_steer"],
        "garage_heading": "沙地转向",
        "garage_brief": "同一条沙地路，换转向车。沙吃抓地，弯吃半径，一起来。",
        "prediction_heading": "沙地转向，短板在哪？",
        "prediction_hint": "先做预测。打滑的轮胎，方向盘还听使唤吗？",
        "prediction_choices": [
            {"id": "sand_steer.link.compound",
             "text": "两个短板会叠加",
             "feedback": "如果又滑又要转弯，温柔是唯一的解。看里程计和轨迹。"},
            {"id": "sand_steer.link.radius",
             "text": "转弯半径是关键",
             "feedback": "如果弯道提前量不够，只能绕大圈。看转弯处的轨迹。"},
        ],
        "running_heading": "沙地转向中",
        "running_normal": "RO-2：沙地转向中，提前量加倍。",
        "running_detected": "RO-2：前方状态已确认。",
        "debrief_success_title": "沙地转向通过",
        "debrief_success_summary": "沙加转向都过去了：胎要对，弯要早。",
        "debrief_failure_summary": "看看是哪一项评分丢了，沙地转向最怕又急又滑。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "沙地转向",
        "debrief_failure_titles": {
            "collision-static": "运送失败：碰到了障碍",
            "collision-dynamic": "运送失败：与叉车发生碰撞",
            "timeout": "运送失败：没有在时限内到达",
        },
        "story_before": "洛一：沙地那条路，让 RO-2 也走一遍。",
        "story_after": "RO-2：到了。沙地上，方向盘也要提前打。",
    },
    {
        "id": "wet_scrub",
        "template": "reach",
        "order": 19,
        "chapter": 2,
        "title": "湿滑履带",
        "map": "research_base",
        "prerequisites": ["sand_steer"],
        "seed": 263,
        "robot": "robot.ro3",
        "spawn": "robot_bay",
        "goal": "glass_door",
        "goal_label": "玻璃门",
        "goal_visual": "stop_zone",
        "time_limit_s": 150,
        "budget": 5,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "trio",
        "arrive_tolerance_m": 0.62,
        "safe_min_clearance_m": 0.2,
        "max_localization_error_m": 3.0,
        "fast_time_s": 40.0,
        "efficient_energy_wh": 0.5,
        "concept": "wet_scrub",
        "prediction_prompt": "湿滑加履带，谁先顶不住？",
        "knowledge_card": "湿滑区里搓履带：水让抓地没了，搓让方向没了，慢是唯一的舵。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.wet_scrub"],
        "garage_heading": "湿滑履带",
        "garage_brief": "车库到玻璃门，蹭过湿滑区边缘，换履带车。滑上加搓，慢字当头。",
        "prediction_heading": "湿滑履带，短板在哪？",
        "prediction_hint": "先做预测。水和履带，哪个先让定位漂掉？",
        "prediction_choices": [
            {"id": "wet_scrub.link.both",
             "text": "两个一起漂",
             "feedback": "如果又湿又搓，里程计最后差几米。看复盘里的定位误差。"},
            {"id": "wet_scrub.link.slow",
             "text": "慢就能过去",
             "feedback": "如果温柔驾驶能到，说明短板在脚不在车。看对比。"},
        ],
        "running_heading": "湿滑履带中",
        "running_normal": "RO-3：湿滑区边缘行驶中。",
        "running_detected": "RO-3：前方状态已确认。",
        "debrief_success_title": "湿滑履带通过",
        "debrief_success_summary": "水加履带都过去了：慢，就是履带的舵。",
        "debrief_failure_summary": "看看是哪一项评分丢了，湿滑履带最怕急。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "湿滑履带",
        "debrief_failure_titles": {
            "collision-static": "履带失败：碰到了障碍",
            "collision-dynamic": "履带失败：与叉车发生碰撞",
            "timeout": "履带失败：没有在时限内到达",
        },
        "story_before": "洛一：湿滑区边上，让 RO-3 蹭过去看看。",
        "story_after": "RO-3：蹭过去了。水里搓履带，慢是唯一的舵。",
    },
    {
        "id": "long_track",
        "template": "reach",
        "order": 21,
        "chapter": 2,
        "title": "履带长途",
        "map": "research_base",
        "prerequisites": ["long_haul"],
        "seed": 127,
        "robot": "robot.ro3",
        "spawn": "signal_tower",
        "goal": "calibration_pad",
        "goal_label": "标定点",
        "goal_visual": "calibration_pad",
        "time_limit_s": 150,
        "budget": 5,
        "required_slots": ["tires", "sensor", "motor"],
        "columns": "trio",
        "arrive_tolerance_m": 0.62,
        "safe_min_clearance_m": 0.0,
        "max_localization_error_m": 2.0,
        "fast_time_s": 140.0,
        "efficient_energy_wh": 1.7,
        "concept": "tracked_pacing",
        "prediction_prompt": "同一条长途，拐角要什么组合才过得去？",
        "knowledge_card": "货架拐角：careful＋抓地胎缺一不可——单稳风格（eco＋careful）照样摇 150 秒，单换胎（grip＋swift）也过不去，传感器快慢不影响；2×2 全跑过，证据在轨迹里。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "motor": "motor.standard",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "motor": "motor.standard",
            "style": 0.0,
        },
        "rewards": ["ability.long_track"],
        "garage_heading": "履带跑长途",
        "garage_brief": "信号塔回标定点，同一条长途，换 RO-3。拐角考的是组合：风格稳＋胎抓地，缺一不可。",
        "prediction_heading": "履带长途，短板在哪？",
        "prediction_hint": "先做预测。慢车配长途，是稳还是摇摆？",
        "prediction_choices": [
            {"id": "long_track.combo",
             "text": "careful＋抓地胎能到",
             "feedback": "对，唯一通关组合：127.78 秒拿全分。缺一件就摇摆或超时。"},
            {"id": "long_track.style_only",
             "text": "只把风格放稳就行",
             "feedback": "错：eco 胎配 careful 照样摇 150 秒。胎不行，稳也没用。"},
            {"id": "long_track.tires_only",
             "text": "只换抓地胎就行",
             "feedback": "错：grip 胎配 swift 也过不去。脾气不行，胎再抓也没用。"},
        ],
        "running_heading": "履带远征",
        "running_normal": "RO-3：长途巡航中，履带稳定。",
        "running_detected": "RO-3：前方状态已确认。",
        "debrief_success_title": "履带抵达",
        "debrief_success_summary": "同一条长途，履带稳稳开到：慢动态也有春天。",
        "debrief_failure_summary": "看看是哪一项评分丢了，履带最怕忽快忽慢。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "慢动态配速",
        "debrief_failure_titles": {
            "collision-static": "履带失败：碰到了障碍",
            "collision-dynamic": "履带失败：与叉车发生碰撞",
            "timeout": "履带失败：没有在时限内到达",
        },
        "story_before": "洛一：当年摇摆的那个路口，让 RO-3 再走一次。",
        "story_after": "RO-3：到了。这次没摇摆，看门狗记得住。",
    },
    {
        "id": "signal_watch",
        "template": "echo",
        "order": 22,
        "chapter": 2,
        "title": "信号瞭望",
        "map": "research_base",
        "prerequisites": ["signal_hold"],
        "seed": 42,
        "time_limit_s": 150,
        "budget": 8,
        "arrive_tolerance_m": 0.62,
        "via_radius_m": 1.25,
        "safe_min_clearance_m": 0.3,
        "fast_time_s": 100.0,
        "efficient_energy_wh": 2.5,
        "concept": "sensing_rate",
        "prediction_prompt": "同一条走廊，同一班叉车：12 米射程 5Hz 和 5 米射程 20Hz，哪个活下来？",
        "knowledge_card": "看得远不如看得勤：5Hz 下叉车每帧走 0.11 米，确认加外推，窗口全是旧位置。默认配置在走廊东口撞上，推荐配置 91.72 秒拿全分。",
        "garage_heading": "换双眼睛再走",
        "garage_brief": "同一条走廊，同一班叉车，只换雷达。射程翻倍，刷新率砍到四分之一，看看会发生什么。",
        "prediction_heading": "出发前，先做一个判断",
        "prediction_hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
        "prediction_choices": [
            {"id": "signal_watch.far_first",
             "text": "看得远更安全",
             "feedback": "错：12 米射程 5Hz，默认配置在 (17.7, 6.1) 一头撞上叉车。射程只管提前量。"},
            {"id": "signal_watch.frequent_first",
             "text": "看得勤更安全",
             "feedback": "对：5 米射程 20Hz，刷新快才能卡准窗口。推荐配置 91.72 秒拿全分。"},
            {"id": "signal_watch.both_matter",
             "text": "射程和刷新率都重要",
             "feedback": "都重要，但这一关的证据是：射程救不了刷新率——先保刷新，再谈射程。"},
        ],
        "running_heading": "瞭望通道",
        "running_normal": "洛一：导航链路稳定，正在接近信号塔。",
        "running_detected": "洛一：锁定叉车，正在预测它的运动轨迹。",
        "debrief_success_title": "任务完成：瞭望确认",
        "debrief_success_summary": "看得勤，走得准，洛一同样送达。",
        "debrief_failure_summary": "选择右侧证据，现场会跳到对应时刻。",
        "debrief_evidence_heading": "因果证据",
        "debrief_evidence_caption": "这些内容由本次仿真的事件与状态生成。",
        "debrief_knowledge_title": "感知刷新率",
        "debrief_failure_titles": {
            "collision-dynamic": "任务失败：洛一与叉车发生碰撞",
            "collision-static": "任务失败：洛一碰到了固定障碍",
            "timeout": "任务失败：没有在时限内送达",
        },
        "default_build": {
            "tires": "tires.grip",
            "sensor": "sensor.long_slow_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "rewards": ["ability.signal_watch"],
        "story_before": "洛一：同一条路，同一班车，换双看得慢的眼睛走一次。",
        "story_after": "洛一：走廊东口，5Hz 没跟上。看得勤，才走得准。",
    },








    {
        "id": "signal_cross",
        "template": "reach",
        "order": 23,
        "chapter": 2,
        "title": "交叉路口",
        "map": "research_base",
        "prerequisites": ["signal_hold"],
        "seed": 11,
        "spawn": "east_gate",
        "goal": "signal_tower",
        "goal_label": "信号塔",
        "goal_visual": "signal_tower",
        "time_limit_s": 150,
        "budget": 8,
        "arrive_tolerance_m": 0.62,
        "via_radius_m": 1.25,
        "safe_min_clearance_m": 0.6,
        "fast_time_s": 100.0,
        "efficient_energy_wh": 2.5,
        "concept": "crossing_phase",
        "prediction_prompt": "南北向的叉车横穿你的去路：它是先过，还是你先过？",
        "knowledge_card": "交叉路口看相位：叉车从侧面来，creep 蹭不过去，等它过去再进路口。等待发生在路口外，不是路口里。",
        "default_build": {
            "tires": "tires.grip",
            "sensor": "sensor.standard_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "rewards": ["ability.signal_cross"],
        "garage_heading": "过路口",
        "garage_brief": "南北向叉车横穿去路。和走廊封锁不一样：威胁从侧面来，等的位置和进入的时机都要算。",
        "prediction_heading": "过路口，先做一个判断",
        "prediction_hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
        "prediction_choices": [
            {"id": "signal_cross.phase_first",
             "text": "看相位：它过去我再进",
             "feedback": "对。交叉路口是相位问题：等侧面威胁通过，窗口出现再进。"},
            {"id": "signal_cross.creep_first",
             "text": "慢慢蹭过去",
             "feedback": "错：侧面来的叉车不会给你蹭的空间，原地等和蹭都会进它的路线。"},
        ],
        "running_heading": "通过交叉路口",
        "running_normal": "洛一：接近交叉路口，确认侧面动态。",
        "running_detected": "洛一：锁定侧面叉车，正在预测它的运动轨迹。",
        "debrief_success_title": "路口通过",
        "debrief_success_summary": "相位算准了：等它过去，再进路口。",
        "debrief_failure_summary": "看看是哪一项评分丢了，交叉路口最怕进得太早。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "交叉相位",
        "debrief_failure_titles": {
            "collision-static": "任务失败：碰到了障碍",
            "collision-dynamic": "任务失败：与侧面叉车发生碰撞",
            "timeout": "任务失败：没有在时限内送达",
        },
        "story_before": "洛一：这条路和走廊不一样，叉车从侧面来。我们等它过去。",
        "story_after": "洛一：过去了。路口是让出来的，不是蹭出来的。",
    },

    {
        "id": "town_freight",
        "template": "haul",
        "order": 24,
        "chapter": 3,
        "title": "城镇首运",
        "map": "signal_town",
        "prerequisites": ["signal_cross"],
        "seed": 7,
        "spawn": "town_gate",
        "via": "energy_hub",
        "via_label": "能源中转",
        "goal": "market_gate",
        "goal_label": "集市门",
        "goal_visual": "calibration_pad",
        "time_limit_s": 180,
        "budget": 8,
        "arrive_tolerance_m": 0.62,
        "via_radius_m": 1.25,
        "safe_min_clearance_m": 0.3,
        "fast_time_s": 150.0,
        "efficient_energy_wh": 3.0,
        "concept": "town_haul",
        "prediction_prompt": "新镇子第一单：湿滑仓门、砂石主街、沙地货道，两辆巡逻车横竖穿行。走哪条道，什么时候让？",
        "knowledge_card": "城镇跑单是综合题：湿滑区提前减速保抓地，主街看巡逻相位等窗口，沙地温柔给油。三段路三种开法，一套莽法全丢分。",
        "default_build": {
            "tires": "tires.eco",
            "sensor": "sensor.long_slow_lidar",
            "computer": "computer.light",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 1.0,
        },
        "recommended_build": {
            "tires": "tires.grip",
            "sensor": "sensor.short_fast_lidar",
            "computer": "computer.performance",
            "motor": "motor.standard",
            "imu": "imu.basic",
            "style": 0.0,
        },
        "rewards": ["ability.town_freight"],
        "garage_heading": "进镇跑单",
        "garage_brief": "毕业了，镇子第一单：镇门取货，能源中转经停，送到集市门。两辆车在街上跑，砂石沙地水全有。",
        "prediction_heading": "首运，怎么跑？",
        "prediction_hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
        "prediction_choices": [
            {"id": "town_freight.link.phase",
             "text": "看相位：巡逻车过去再进街",
             "feedback": "对：主街是相位问题，等横向巡逻通过再进。看等待事件和最小间隙。"},
            {"id": "town_freight.link.power",
             "text": "马力大就能碾过去",
             "feedback": "错：湿滑加沙地，大马力只会刨坑打滑，看能耗表和里程计。"},
            {"id": "town_freight.link.static",
             "text": "把巡逻车记进静态地图",
             "feedback": "错：巡逻车会动，静态地图只管房子不管车，下一刻位置得靠预测。"},
        ],
        "running_heading": "城镇首运",
        "running_normal": "洛一：进镇了，正在去能源中转。",
        "running_detected": "洛一：锁定巡逻车，正在预测它的运动轨迹。",
        "debrief_success_title": "首运送达",
        "debrief_success_summary": "三段路，三种开法，一单送达：镇子欢迎你。",
        "debrief_failure_summary": "看看是哪一项评分丢了，城镇综合题最怕只会一种开法。",
        "debrief_evidence_heading": "本次驾驶",
        "debrief_evidence_caption": "三项评分分别对应安全、迅速与节能，来自真实仿真。",
        "debrief_knowledge_title": "城镇综合",
        "debrief_failure_titles": {
            "collision-static": "运送失败：撞上了镇子建筑",
            "collision-dynamic": "运送失败：与巡逻车发生碰撞",
            "timeout": "运送失败：没有在时限内送达",
        },
        "story_before": "洛一：毕业了！镇子第一单，听说主街上有车，弯道上有沙。慢慢开，准时到。",
        "story_after": "洛一：首运拿下。镇长说，以后镇子的货都归我们了。",
    },

]

SAND_COLUMNS = [
    {"slot": "tires", "title": "驱动轮胎",
     "options": ["tires.standard", "tires.grip", "tires.eco", "tires.sand"]},
    {"slot": "sensor", "title": "距离雷达",
     "options": ["sensor.short_fast_lidar", "sensor.standard_lidar", "sensor.long_slow_lidar"]},
    {"slot": "motor", "title": "电机",
     "options": ["motor.standard", "motor.torque", "motor.eco"]},
]

TRIO_COLUMNS = [
    {"slot": "tires", "title": "驱动轮胎",
     "options": ["tires.standard", "tires.grip", "tires.eco"]},
    {"slot": "sensor", "title": "距离雷达",
     "options": ["sensor.short_fast_lidar", "sensor.standard_lidar", "sensor.long_slow_lidar"]},
    {"slot": "motor", "title": "电机",
     "options": ["motor.standard", "motor.torque", "motor.eco"]},
]

COLUMNS = [
    {"slot": "tires", "title": "驱动轮胎",
     "options": ["tires.standard", "tires.grip", "tires.eco", "tires.sand", "tires.slick"]},
    {"slot": "sensor", "title": "距离雷达",
     "options": ["sensor.short_fast_lidar", "sensor.standard_lidar", "sensor.long_slow_lidar",
                 "sensor.precision_lidar"]},
    {"slot": "computer", "title": "计算单元",
     "options": ["computer.light", "computer.mid", "computer.performance"]},
    {"slot": "motor", "title": "电机",
     "options": ["motor.standard", "motor.torque", "motor.eco"]},
    {"slot": "imu", "title": "惯性单元",
     "options": ["imu.basic", "imu.precise"]},
]


def check_map(spec: dict) -> None:
    data = json.loads((ROOT / "assets" / "maps" / f"{spec['map']}.json").read_text())
    if spec["template"] == "echo":
        markers = ["robot_bay", "supply_station", "signal_tower"]
    elif spec["template"] == "haul":
        markers = [spec["spawn"], spec["via"], spec["goal"]]
    else:
        markers = [spec["spawn"], spec.get("via"), spec["goal"]]
    for marker in markers:
        if marker:
            assert marker in data["markers"], f"map missing marker {marker}"
    if spec["template"] in ("echo", "haul"):
        need = REQUIRE_MAP[spec["map"]]
        active = {a["id"]: a.get("active_in", []) for a in data.get("actors", [])}
        for actor in need["actors"]:
            assert spec["id"] in active.get(actor, []), f"actor {actor} not active in {spec['id']}"


def build_objective(spec: dict) -> dict:
    if spec["template"] == "echo":
        return {
            "type": "deliver_via",
            "spawn": "robot_bay",
            "via": "supply_station",
            "via_label": "能源模块",
            "goal": "signal_tower",
            "goal_label": "信号塔",
            "goal_visual": "signal_tower",
            "time_limit_s": spec["time_limit_s"],
            "max_collisions": 0,
            "arrive_tolerance_m": spec["arrive_tolerance_m"],
            "via_radius_m": spec["via_radius_m"],
        }
    if spec["template"] == "haul":
        return {
            "type": "deliver_via",
            "spawn": spec["spawn"],
            "via": spec["via"],
            "via_label": spec["via_label"],
            "goal": spec["goal"],
            "goal_label": spec["goal_label"],
            "goal_visual": spec.get("goal_visual", "calibration_pad"),
            "time_limit_s": spec["time_limit_s"],
            "max_collisions": 0,
            "arrive_tolerance_m": spec["arrive_tolerance_m"],
            "via_radius_m": spec.get("via_radius_m", 1.25),
        }
    if spec["template"] == "reach":
        return {
            "type": "reach",
            "spawn": spec["spawn"],
            "goal": spec["goal"],
            "goal_label": spec["goal_label"],
            "goal_visual": spec["goal_visual"],
            "time_limit_s": spec["time_limit_s"],
            "max_collisions": 0,
            "arrive_tolerance_m": spec["arrive_tolerance_m"],
        }
    raise AssertionError(f"unknown template {spec['template']}")


def build_level(spec: dict) -> dict:
    return {
        "schema_version": 1,
        "id": spec["id"],
        "order": spec["order"],
        "title": spec["title"],
        "map": spec["map"],
        "control_mode": "autonomous",
        "localization_mode": "fused",
        "chapter": spec["chapter"],
        "prerequisites": spec["prerequisites"],
        "seed": spec["seed"],
        "objective": build_objective(spec),
        "loadout_rules": {
            "budget": spec["budget"],
            "required_slots": spec.get(
                "required_slots", ["tires", "sensor", "computer", "motor", "imu"]),
        },
        "teaching": {
            "concept": spec["concept"],
            "prediction_prompt": spec["prediction_prompt"],
            "knowledge_card": spec["knowledge_card"],
        },
        "score": {
            "safe_min_clearance_m": spec["safe_min_clearance_m"],
            **({"max_localization_error_m": spec["max_localization_error_m"]}
               if "max_localization_error_m" in spec else {}),
            "fast_time_s": spec["fast_time_s"],
            "efficient_energy_wh": spec["efficient_energy_wh"],
        },
        "rewards": spec["rewards"],
    }


def build_presentation(spec: dict) -> dict:
    doc: dict = {
        "schema_version": 1,
        "id": f"presentation.{spec['id']}",
        "is_default": False,
        "level": spec["id"],
    }
    if spec.get("robot"):
        doc["robot"] = spec["robot"]
    if spec.get("chassis_options"):
        doc["chassis_options"] = list(spec["chassis_options"])
    doc["garage"] = {
        "heading": spec["garage_heading"],
        "brief": spec["garage_brief"],
        "component_columns": copy.deepcopy(
            {"trio": TRIO_COLUMNS, "sand": SAND_COLUMNS}.get(spec.get("columns"), COLUMNS)),
        "policy_title": "驾驶风格",
        "policies": ["policy.careful", "policy.balanced", "policy.swift"],
        "default_build": spec["default_build"],
        "recommended_build": spec["recommended_build"],
    }
    doc["prediction"] = {
        "heading": spec["prediction_heading"],
        "hint": spec["prediction_hint"],
        "choices": copy.deepcopy(spec["prediction_choices"]),
    }
    doc["running"] = {
        "heading": spec["running_heading"],
        "normal_line": spec["running_normal"],
        "detected_line": spec["running_detected"],
    }
    doc["debrief"] = {
        "success_title": spec["debrief_success_title"],
        "success_summary": spec["debrief_success_summary"],
        "failure_summary": spec["debrief_failure_summary"],
        "evidence_heading": spec["debrief_evidence_heading"],
        "evidence_caption": spec["debrief_evidence_caption"],
        "knowledge_title": spec["debrief_knowledge_title"],
        "failure_titles": copy.deepcopy(spec["debrief_failure_titles"]),
    }
    return doc


def build_beats(spec: dict) -> list[dict]:
    return [
        {"id": f"{spec['id']}.before", "level": spec["id"], "when": "before",
         "speaker": "洛一", "text": spec["story_before"]},
        {"id": f"{spec['id']}.after", "level": spec["id"], "when": "after",
         "speaker": "洛一", "text": spec["story_after"]},
    ]


def render(data: dict) -> str:
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def dump(path: Path, data: dict, dry_run: bool) -> bool:
    # Check mode never touches the tree: a gate that rewrites what it checks
    # is not a gate. (It once ate a hand-maintained chapter field.)
    if dry_run:
        return not path.exists() or path.read_text() != render(data)
    text = render(data)
    if path.exists() and path.read_text() == text:
        return False
    path.write_text(text)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    changed: list[str] = []

    for spec in LEVELS:
        assert spec["template"] in ("echo", "reach", "haul"), spec["template"]
        check_map(spec)
        if dump(ROOT / "assets" / "levels" / f"{spec['id']}.json", build_level(spec),
                 args.check):
            changed.append(f"levels/{spec['id']}.json")
        if dump(ROOT / "assets" / "presentation" / f"{spec['id']}.json",
                 build_presentation(spec), args.check):
            changed.append(f"presentation/{spec['id']}.json")

    story_path = ROOT / "assets" / "story" / "chapter_01.json"
    story = json.loads(story_path.read_text())
    have = {beat["id"] for beat in story["beats"]}
    for spec in LEVELS:
        for beat in build_beats(spec):
            if beat["id"] not in have:
                story["beats"].append(beat)
                have.add(beat["id"])
                changed.append(f"story/{beat['id']}")
    if any(b["id"].startswith("signal_echo.") for b in story["beats"]):
        story_text = json.dumps(story, ensure_ascii=False, indent=2) + "\n"
        if story_path.read_text() != story_text:
            story_path.write_text(story_text)

    campaign_path = ROOT / "assets" / "campaign.json"
    campaign = json.loads(campaign_path.read_text())
    for spec in LEVELS:
        if spec["id"] not in campaign["levels"]:
            campaign["levels"].append(spec["id"])
            changed.append(f"campaign/{spec['id']}")
    campaign_text = json.dumps(campaign, ensure_ascii=False, indent=2) + "\n"
    if campaign_path.read_text() != campaign_text:
        campaign_path.write_text(campaign_text)

    if args.check and changed:
        print("stale generated content:", ", ".join(changed))
        return 1
    print("generated:", ", ".join(changed) if changed else "up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
