#include "game/campaign.hpp"
#include "game/campaign_play.hpp"
#include <SFML/System/String.hpp>
#include <cstdio>
#include <cstring>
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
int main(){
  using namespace camp;
  // 1. 关卡表: 7关, task id 1..7连续, 每关三线索+知识卡齐全
  { auto& ls = levels();
    REQ(ls.size() == 7);
    for (size_t i = 0; i < ls.size(); i++) {
      REQ(ls[i].task == (int)i + 1);
      REQ(std::strlen(ls[i].clue1) > 0 && std::strlen(ls[i].clue2) > 0 && std::strlen(ls[i].clue3) > 0);
      REQ(std::strlen(ls[i].card.id) > 0 && std::strlen(ls[i].card.title) > 0 && std::strlen(ls[i].card.body) > 0);
      REQ(std::strlen(ls[i].story_before) > 0 && std::strlen(ls[i].story_after) > 0);
    }
    printf("[ok] levels=7 cards+clues complete\n"); }
  // 2. 初始只解锁第1关; 完成逐关解锁; 失败/锁定完成无效
  { CampaignState c;
    REQ(c.unlocked(1) && !c.unlocked(2));
    REQ(!c.complete(2, true));   // 锁定关完成被忽略
    REQ(!c.complete(1, false));  // 失败不解锁
    REQ(!c.unlocked(2));
    REQ(c.complete(1, true));
    REQ(c.unlocked(2) && !c.unlocked(3));
    REQ(!c.complete(1, true));   // 重复完成幂等false
    printf("[ok] progression unlocks\n"); }
  // 3. 全通: progress=7, 知识卡7张按序
  { CampaignState c;
    for (int t = 1; t <= 7; t++) REQ(c.complete(t, true));
    REQ(c.progress() == 7);
    auto cards = c.cards();
    REQ(cards.size() == 7);
    REQ(!std::strcmp(cards[0].id, "pid-vmax") && !std::strcmp(cards[6].id, "fog-replan"));
    printf("[ok] full run cards=7\n"); }
  // 4. 存档位图往返+前缀闭合校验
  { CampaignState a, b;
    a.complete(1, true); a.complete(2, true);
    REQ(b.loadMask(a.mask()));
    REQ(b.progress() == 2 && b.unlocked(3) && !b.unlocked(4));
    REQ(!b.loadMask(1u << 6));  // 跳关读档拒绝(第7关无前缀)
    REQ(!b.loadMask(1u << 30)); // 越界位拒绝
    REQ(b.loadMask(0) && b.progress() == 0);  // 空档合法(清零)
    printf("[ok] mask roundtrip\n"); }
  // 5. 线索分级: fails越多越显式; 无效关返回空串
  { REQ(clueFor(6, 0) != nullptr && std::strlen(clueFor(6, 0)) > 0);
    REQ(clueFor(6, 5) == levels()[5].clue3);
    REQ(clueFor(6, -1) == levels()[5].clue1);
    REQ(std::strlen(clueFor(99, 0)) == 0 && findLevel(99) == nullptr);
    printf("[ok] clue tiers\n"); }
  // 6. Play状态机: 选关/门禁/失败升级线索/成功自动跳关
  { Play p;
    REQ(p.selected() == 1 && p.canRunSelected());
    REQ(!p.canRun(2));
    REQ(!p.select(99));  // 未定义关拒绝
    p.select(2);
    REQ(p.selected() == 2);  // 选中允许(跑不跑由canRun gate)
    REQ(!p.canRunSelected());
    p.select(1);
    p.applyResult(1, false);
    REQ(p.fails(1) == 1 && !p.state().completed(1));
    REQ(p.clueSelected() == levels()[0].clue2);  // 失败1次->tier2
    p.applyResult(1, false);
    REQ(p.clueSelected() == levels()[0].clue3);  // 失败2次->tier3
    REQ(p.applyResult(1, true));
    REQ(p.state().completed(1) && p.selected() == 2 && p.canRunSelected());
    REQ(!p.applyResult(9, true) && !p.applyResult(-1, false));  // 越界安全
    printf("[ok] play select/gate/clue/fs\n"); }
  // 7. 标准答案loadout与clue3同源: L3/L6/L7特化, 其余默认
  { auto d = suggestedLoadout(1);
    REQ(d.has_lidar && !d.use_dwa && !d.fog && !d.use_odom);
    auto o = suggestedLoadout(3);
    REQ(o.use_odom && o.slip_scale == 0.2);
    auto w = suggestedLoadout(6);
    REQ(w.use_dwa);
    auto f = suggestedLoadout(7);
    REQ(f.fog && f.unknown_free && f.replan);
    printf("[ok] suggested loadouts\n"); }
  // 8. merge玩家调参: 默认值=答案(行为不变), 只动三项, 关卡项不串台
  { sim::PID pid(2.0, 0.2, 0.15, 0.6, 2.0);
    sim::DwaWeights dwt{1.0, 2.0, 0.3};
    auto m = mergePlayerTuning(suggestedLoadout(6), pid, dwt, 1.5);
    REQ(m.use_dwa && m.kp == 2.0 && m.dwa.w_clear == 2.0 && m.h_weight == 1.5);
    sim::PID zero(0, 0, 0, 0.6, 2.0);
    auto z = mergePlayerTuning(suggestedLoadout(1), zero, dwt, 1.5);
    REQ(z.kp == 0 && z.ki == 0 && z.kd == 0);
    REQ(z.has_lidar && !z.use_dwa);  // 关卡项原样保留
    printf("[ok] merge tuning\n"); }
  // 8. UTF-8回归: SFML2的std::string隐转按ANSI解码, C locale下中文逐字节误读
  // (战役面板tofu根因). 只测纯字符串转换, 不建GL上下文, headless可跑.
  { sf::String u = sf::String::fromUtf8("醒来", "醒来" + 6);
    REQ(u.getSize() == 2);
    REQ(u[0] == 0x9192 && u[1] == 0x6765);
    printf("[ok] utf8 decode\n"); }
  // 9. 失败原因: 记录/清空/专属hint全覆盖runner的fail_reason集合
  { const char* reasons[] = {"collision-blind", "collision", "collision-dyn", "no-path",
                             "via-missed", "timeout-no-detect", "timeout", "lost-drift"};
    for (auto r : reasons) REQ(std::strlen(hintFor(r)) > 0);
    REQ(std::strlen(hintFor("")) == 0 && std::strlen(hintFor(nullptr)) == 0);
    REQ(std::strlen(hintFor("no-such-reason")) == 0);
    Play p;
    p.applyResult(2, false, "collision-blind");
    REQ(p.fails(2) == 1 && !std::strcmp(p.failReason(2), "collision-blind"));
    REQ(p.hintSelected() != nullptr);
    p.select(2);
    REQ(std::strlen(p.hintSelected()) > 0);
    p.applyResult(2, true);  // 成功清空失败原因
    REQ(p.failReason(2)[0] == '\0' && std::strlen(p.hintSelected()) == 0);
    // 超长原因截断不断裂
    char big[256];
    std::memset(big, 'x', sizeof big - 1);
    big[sizeof big - 1] = '\0';
    p.applyResult(3, false, big);
    REQ(std::strlen(p.failReason(3)) == 31);
    printf("[ok] fail reasons + hints\n"); }
  printf("ALL CAMPAIGN TESTS PASSED\n");
  return 0;
}
