#include "game/campaign.hpp"
#include "game/campaign_play.hpp"
#include "game/task.hpp"
#include <cstdio>
#include <cstring>
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
// M6新手全通审计: 按战役顺序, 用每关clue3指定的"标准答案"loadout跑一遍.
// 断言: 每关能过、用时留有余量(<80%时限)、战役解锁链同步推进、知识卡按序掉落.
int main(){
  using namespace task;
  using namespace camp;
  auto ts = allTasks();
  auto& ls = levels();
  REQ(ts.size() == ls.size());
  CampaignState c;
  printf("%-8s %-12s %6s %8s %8s %s\n", "task", "name", "t_used", "t_limit", "margin", "card");
  for (size_t i = 0; i < ts.size(); i++) {
    int t = ts[i].id;
    REQ(c.unlocked(t));  // 战役门: 前一关已过才能进这一关
    Loadout lo = suggestedLoadout(t);  // 标准答案与clue3同源, 单一事实来源
    Metrics m = run_task(ts[i], lo);
    double margin = m.time_used / ts[i].time_limit;
    const Level* l = findLevel(t);
    printf("%-8d %-12s %6.1f %8.1f %7.0f%% %s\n", t, l->name, m.time_used,
           ts[i].time_limit, margin * 100, l->card.id);
    if (!m.success) {
      // 失败时给玩家看的正是这条分级线索, 必须非空
      printf("  fail=%s clue1=%s\n", m.fail_reason, clueFor(t, 0));
      REQ(std::strlen(clueFor(t, 2)) > 0);
    }
    REQ(m.success);
    REQ(margin < 0.8);  // 新手余量: 标准答案用时不得超过时限八成
    REQ(c.complete(t, true));
  }
  REQ(c.progress() == 7);
  auto cards = c.cards();
  REQ(cards.size() == 7);
  for (size_t i = 0; i < ls.size(); i++) REQ(!std::strcmp(cards[i].id, ls[i].card.id));
  // 反面演示必败: 每关badLoadout都失败(失败分支UI的数据地基)
  for (size_t i = 0; i < ts.size(); i++) {
    Loadout bad = badLoadout(ts[i].id);
    Metrics b = run_task(ts[i], bad);
    printf("  [bad L%d] ok=%d why=%s\n", ts[i].id, (int)b.success, b.fail_reason);
    REQ(!b.success);
  }
  printf("ALL PLAYTHROUGH TESTS PASSED (7/7 novice full clear)\n");
  return 0;
}
