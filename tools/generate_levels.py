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
    }
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
        "fast_time_s": 60.0,
        "efficient_energy_wh": 1.6,
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
        "fast_time_s": 32.0,
        "efficient_energy_wh": 0.55,
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
    else:
        markers = [spec["spawn"], spec.get("via"), spec["goal"]]
    for marker in markers:
        if marker:
            assert marker in data["markers"], f"map missing marker {marker}"
    if spec["template"] == "echo":
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
    doc["garage"] = {
        "heading": spec["garage_heading"],
        "brief": spec["garage_brief"],
        "component_columns": copy.deepcopy(
            TRIO_COLUMNS if spec.get("columns") == "trio" else COLUMNS),
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
        assert spec["template"] in ("echo", "reach"), spec["template"]
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
