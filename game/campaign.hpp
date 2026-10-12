#pragma once
#include <cstdint>
#include <vector>
// M5 campaign: 纯数据+状态机, 无图形. 关卡顺序与task id(1..7)对齐.
// 解锁规则: 第1关初始解锁; 完成第N关(success=true)即解锁第N+1关.
// 线索分级: fails=0/1/2+ 对应 tier1(方向)/tier2(概念)/tier3(显式参数).
namespace camp {
struct Card {
  const char* id;
  const char* title;
  const char* body;
};
struct Level {
  int task{0};
  const char* name{""};
  const char* story_before{""};  // 进关前剧情
  const char* story_after{""};   // 通关后剧情
  const char* brief{""};         // 任务简报(目标+约束)
  const char* clue1{""};         // 方向性提示
  const char* clue2{""};         // 概念提示
  const char* clue3{""};         // 显式参数提示
  Card card;                     // 通关掉落的知识卡
};

const std::vector<Level>& levels();  // 7关, 与task::allTasks()同序
const Level* findLevel(int task);    // 无效task返回nullptr
const char* clueFor(int task, int fails);  // fails<0按0计, >2按tier3

class CampaignState {
 public:
  CampaignState() { reset(); }
  void reset();
  size_t total() const;
  bool unlocked(int task) const;   // 未定义task返回false
  bool completed(int task) const;
  // success=true且已解锁时标记完成(幂等), 返回是否首次完成; 锁定/失败返回false
  bool complete(int task, bool success);
  std::vector<Card> cards() const;  // 已获知识卡(按关卡序)
  int progress() const;             // 已完成关数
  uint32_t mask() const;            // 存档位图: bit(task-1)
  bool loadMask(uint32_t m);        // 读档: 只接受total位内的掩码, 非法返回false
 private:
  uint32_t done_{0};
};
}  // namespace camp
