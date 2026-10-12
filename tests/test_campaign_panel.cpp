#include "view/campaign_panel.hpp"
#include "game/campaign_play.hpp"
#include <SFML/Graphics.hpp>
#include <cstdio>
#include <cstdlib>
#define REQ(cond) do{ if(!(cond)){ printf("[FAIL] %s:%d: %s\n",__FILE__,__LINE__,#cond); return 1; } }while(0)
// DISPLAY门控: RenderTexture要GL上下文, 无屏CI直接skip(返回0).
// 锁: 面板离屏渲染链路(选关/简报/失败行/toast) + CJK字体真实生效(与DejaVu输出不同).
static const char* CJK = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc";
static const char* DEJ = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf";

static sf::Image render(CampaignPanel& p) {
  sf::RenderTexture rt;
  if (!rt.create(360, 720)) {
    printf("[FAIL] RenderTexture create (no GL?)\n");
    std::exit(1);
  }
  rt.clear(sf::Color::Black);
  p.draw(rt);
  rt.display();
  return rt.getTexture().copyToImage();
}
static long inkIn(const sf::Image& im, unsigned y0, unsigned y1) {
  long n = 0;
  for (unsigned y = y0; y < y1 && y < im.getSize().y; y++)
    for (unsigned x = 0; x < im.getSize().x; x++) {
      auto px = im.getPixel(x, y);
      if (px.r > 40 || px.g > 40 || px.b > 40) n++;
    }
  return n;
}
static long diffPx(const sf::Image& a, const sf::Image& b) {
  if (a.getSize() != b.getSize()) return -1;
  long n = 0;
  for (unsigned y = 0; y < a.getSize().y; y++)
    for (unsigned x = 0; x < a.getSize().x; x++) {
      auto pa = a.getPixel(x, y), pb = b.getPixel(x, y);
      if (pa.r != pb.r || pa.g != pb.g || pa.b != pb.b) n++;
    }
  return n;
}
static void fill(CampaignPanel& p, camp::CampaignState& st) {
  auto& ls = camp::levels();
  p.setVisible(true);
  p.setLevels(ls, st, 2);
  const camp::Level* l = camp::findLevel(2);
  p.setDetail(l->story_before, l->brief, l->clue1, 0);
  p.setLastFail("collision-dyn", camp::hintFor("collision-dyn"));
}
int main() {
  if (!std::getenv("DISPLAY")) {
    printf("[skip] no DISPLAY, panel render test needs GL\n");
    return 0;
  }
  CampaignPanel probe;
  if (!probe.initFromFile(CJK)) {
    printf("[skip] no CJK font at %s\n", CJK);
    return 0;
  }
  camp::CampaignState st;
  st.complete(1, true);
  // 1. 各带都有墨: 标题/列表/简报(含中文)/失败行
  CampaignPanel p;
  REQ(p.init());
  fill(p, st);
  p.setToast("L1 done");
  sf::Image im = render(p);
  REQ(inkIn(im, 8, 32) > 50);     // 标题行
  REQ(inkIn(im, 32, 180) > 200);  // 关卡列表
  REQ(inkIn(im, 180, 560) > 500);  // 简报+线索+失败行(中文区)
  printf("[ok] bands have ink\n");
  // 2. CJK与DejaVu输出必须不同(证明CJK字形真实生效, 非fallback tofu)
  CampaignPanel q;
  REQ(q.initFromFile(DEJ));
  fill(q, st);
  q.setToast("L1 done");
  sf::Image im2 = render(q);
  long d = diffPx(im, im2);
  REQ(d > 1000);
  printf("[ok] cjk differs from dejavu by %ld px\n", d);
  // 3. toast变化引起画面变化(防陈旧帧/绘制跳过)
  p.setToast("");
  sf::Image im3 = render(p);
  REQ(diffPx(im, im3) > 50);
  printf("[ok] toast dynamism\n");
  printf("ALL PANEL RENDER TESTS PASSED\n");
  return 0;
}
