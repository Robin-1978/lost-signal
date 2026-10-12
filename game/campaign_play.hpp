#pragma once
#include "campaign.hpp"
#include "task.hpp"
#include <cstring>
// M5 glue(Core,无图形): 把clue3变成机器可读的标准答案, 把选关/失败计数/结算收成可测状态机.
// app只做渲染+后台线程调run_task, 结算逻辑全在这里.
namespace camp {
// 每关clue3对应的标准答案loadout(与test_playthrough同源, 单一事实来源).
inline task::Loadout suggestedLoadout(int t) {
  task::Loadout lo;  // 默认好配置
  if (t == 3) {
    lo.use_odom = true;
    lo.slip_scale = 0.2;
  } else if (t == 6) {
    lo.use_dwa = true;
  } else if (t == 7) {
    lo.fog = true;
    lo.unknown_free = true;
    lo.replan = true;
  }
  return lo;
}

// 每关的反面演示loadout(必败, 与各关对比实验同源, 给玩家看失败+诊断用).
inline task::Loadout badLoadout(int t) {
  task::Loadout lo;
  if (t == 1 || t == 3 || t == 5) {
    lo.use_odom = true;  // 烂胎估计跟随: 漂移跟丢
    lo.slip_scale = 3.0;
  } else if (t == 2) {
    lo.has_lidar = false;  // 瞎子直行: collision-blind
  } else if (t == 4) {
    lo.inflation = 1.5;  // 过度膨胀: 封死无路
  } else if (t == 6) {
    lo.use_dwa = false;  // 致盲直穿: 被叉车撞
  } else if (t == 7) {
    lo.fog = true;  // 开环进雾: 必撞墙
    lo.unknown_free = true;
    lo.replan = false;
  }
  return lo;
}

// 失败原因专属hint(run_task的fail_reason直通, 未知原因回空串).
// 与三级线索正交: tier管"说多细", hint管"往哪看".
inline const char* hintFor(const char* reason) {
  if (!reason || !reason[0]) return "";
  if (!std::strcmp(reason, "collision-blind")) return "瞎跑必撞: 先确认has_lidar=true再谈别的.";
  if (!std::strcmp(reason, "collision")) return "撞静态墙: 加inflation或降cruise, 别硬挤.";
  if (!std::strcmp(reason, "collision-dyn")) return "被叉车撞: 开use_dwa, 让它帮你刹车.";
  if (!std::strcmp(reason, "no-path")) return "无路: 保守模式(未知当墙)容易无解, 切乐观+重规划.";
  if (!std::strcmp(reason, "via-missed")) return "经停Miss: 距via点1m内才算到, 别绕太远.";
  if (!std::strcmp(reason, "timeout-no-detect")) return "没看见: 检查observe_range与检测锥±0.35rad.";
  if (!std::strcmp(reason, "timeout") || !std::strcmp(reason, "lost-drift"))
    return "超时/跟丢: h_w调大提速, 或slip太大漂移丢了路.";
  return "";
}
class Play {
 public:
  Play() : sel_(1) {
    for (int i = 0; i < 8; i++) {
      fails_[i] = 0;
      fail_why_[i][0] = '\0';
    }
  }
  CampaignState& state() { return st_; }
  const CampaignState& state() const { return st_; }
  int selected() const { return sel_; }
  // 只允许移到已定义关卡; 返回是否选中变化
  bool select(int t) {
    if (!findLevel(t)) return false;
    if (sel_ == t) return false;
    sel_ = t;
    return true;
  }
  void selectNext() {
    for (int t = sel_ + 1; t <= 7; t++)
      if (findLevel(t)) {
        sel_ = t;
        return;
      }
  }
  void selectPrev() {
    for (int t = sel_ - 1; t >= 1; t--)
      if (findLevel(t)) {
        sel_ = t;
        return;
      }
  }
  // Enter是否允许跑: 关卡已解锁
  bool canRun(int t) const { return st_.unlocked(t); }
  bool canRunSelected() const { return canRun(sel_); }
  int fails(int t) const { return (t >= 0 && t < 8) ? fails_[t] : 0; }
  const char* failReason(int t) const { return (t >= 0 && t < 8) ? fail_why_[t] : ""; }
  const char* clueSelected() const { return clueFor(sel_, fails_[sel_]); }
  const char* hintSelected() const { return hintFor(fail_why_[sel_]); }
  // 后台run_task回来后结算: 成功->解锁下一关并自动选中它(清空失败原因);
  // 失败->失败数+1(线索升级)+记原因(专属hint). reason可空.
  // 返回成功时是否为首次完成
  bool applyResult(int t, bool success, const char* reason = "") {
    if (t < 0 || t >= 8) return false;
    if (success) {
      bool first = st_.complete(t, true);
      fail_why_[t][0] = '\0';
      // 自动跳到下一未完成且已解锁关
      for (int n = t + 1; n <= 7; n++)
        if (findLevel(n) && st_.unlocked(n) && !st_.completed(n)) {
          sel_ = n;
          break;
        }
      return first;
    }
    fails_[t]++;
    if (reason) {
      std::strncpy(fail_why_[t], reason, sizeof(fail_why_[t]) - 1);
      fail_why_[t][sizeof(fail_why_[t]) - 1] = '\0';
    }
    return false;
  }

 private:
  CampaignState st_;
  int sel_;
  int fails_[8];
  char fail_why_[8][32];
};
}  // namespace camp
