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
        "order": 6,
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
    }
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
    need = REQUIRE_MAP[spec["map"]]
    for marker in need["markers"]:
        assert marker in data["markers"], f"map missing marker {marker}"
    active = {a["id"]: a.get("active_in", []) for a in data.get("actors", [])}
    for actor in need["actors"]:
        assert spec["id"] in active.get(actor, []), f"actor {actor} not active in {spec['id']}"


def build_level(spec: dict) -> dict:
    return {
        "schema_version": 1,
        "id": spec["id"],
        "order": spec["order"],
        "title": spec["title"],
        "map": spec["map"],
        "control_mode": "autonomous",
        "localization_mode": "fused",
        "prerequisites": spec["prerequisites"],
        "seed": spec["seed"],
        "objective": {
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
        },
        "loadout_rules": {
            "budget": spec["budget"],
            "required_slots": ["tires", "sensor", "computer", "motor", "imu"],
        },
        "teaching": {
            "concept": spec["concept"],
            "prediction_prompt": spec["prediction_prompt"],
            "knowledge_card": spec["knowledge_card"],
        },
        "score": {
            "safe_min_clearance_m": spec["safe_min_clearance_m"],
            "fast_time_s": spec["fast_time_s"],
            "efficient_energy_wh": spec["efficient_energy_wh"],
        },
        "rewards": spec["rewards"],
    }


def build_presentation(spec: dict) -> dict:
    return {
        "schema_version": 1,
        "id": f"presentation.{spec['id']}",
        "is_default": False,
        "level": spec["id"],
        "garage": {
            "heading": "换个跑法，再走一次",
            "brief": "同一条叉车通道，预算更紧。洛一已经证明过自己，这次试试不同的动力组合。",
            "component_columns": copy.deepcopy(COLUMNS),
            "policy_title": "驾驶风格",
            "policies": ["policy.careful", "policy.balanced", "policy.swift"],
            "default_build": spec["default_build"],
            "recommended_build": spec["recommended_build"],
        },
        "prediction": {
            "heading": "出发前，先做一个判断",
            "hint": "先记录你的预测。它不影响评分，运行后可以用现场证据验证。",
            "choices": [
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
        },
        "running": {
            "heading": "回响通道",
            "normal_line": "洛一：导航链路稳定，正在接近信号塔。",
            "detected_line": "洛一：锁定叉车，正在预测它的运动轨迹。",
        },
        "debrief": {
            "success_title": "任务完成：回响确认",
            "success_summary": "同一条路，不同的跑法，洛一同样送达。",
            "failure_summary": "选择右侧证据，现场会跳到对应时刻。",
            "evidence_heading": "因果证据",
            "evidence_caption": "这些内容由本次仿真的事件与状态生成。",
            "knowledge_title": "动力与感知组合",
            "failure_titles": {
                "collision-dynamic": "任务失败：洛一与叉车发生碰撞",
                "collision-static": "任务失败：洛一碰到了固定障碍",
                "timeout": "任务失败：没有在时限内送达",
            },
        },
    }


def build_beats(spec: dict) -> list[dict]:
    return [
        {"id": f"{spec['id']}.before", "level": spec["id"], "when": "before",
         "speaker": "洛一", "text": spec["story_before"]},
        {"id": f"{spec['id']}.after", "level": spec["id"], "when": "after",
         "speaker": "洛一", "text": spec["story_after"]},
    ]


def dump(path: Path, data: dict) -> bool:
    text = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
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
        assert spec["template"] == "echo"
        check_map(spec)
        if dump(ROOT / "assets" / "levels" / f"{spec['id']}.json", build_level(spec)):
            changed.append(f"levels/{spec['id']}.json")
        if dump(ROOT / "assets" / "presentation" / f"{spec['id']}.json",
                build_presentation(spec)):
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
