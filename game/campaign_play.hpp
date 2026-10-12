#pragma once
#include "campaign.hpp"
#include "task.hpp"
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

// 选关+失败计数+结算. fails只增不减(读档不带失败数, 避免刷线索).
class Play {
 public:
  Play() : sel_(1) {
    for (int i = 0; i < 8; i++) fails_[i] = 0;
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
  const char* clueSelected() const { return clueFor(sel_, fails_[sel_]); }
  // 后台run_task回来后结算: 成功->解锁下一关并自动选中它; 失败->失败数+1(线索升级)
  // 返回成功时是否为首次完成
  bool applyResult(int t, bool success) {
    if (t < 0 || t >= 8) return false;
    if (success) {
      bool first = st_.complete(t, true);
      // 自动跳到下一未完成且已解锁关
      for (int n = t + 1; n <= 7; n++)
        if (findLevel(n) && st_.unlocked(n) && !st_.completed(n)) {
          sel_ = n;
          break;
        }
      return first;
    }
    fails_[t]++;
    return false;
  }

 private:
  CampaignState st_;
  int sel_;
  int fails_[8];
};
}  // namespace camp
