// M4: app只做输入+渲染, 仿真全在game/PlaygroundSim(固定步长, headless可测).
// M5: N战役面板, [/]选关, Enter跑本关(标准答案), B反面演示(必败配置看诊断).
// 按键: 0/1/2/3/4模式 Q/A Kp W/S Ki E/D Kd | Z/X h_w | T/G Y/H U/J DWA
//       L lidar C slip K kidnap | O雾战 F面板 V跟随 P轨迹 R重置 | N战役 [/]选关 Enter开跑 B反面演示
#include "../game/campaign_play.hpp"
#include "../game/level.hpp"
#include "../game/playground.hpp"
#include "../game/task.hpp"
#include "../view/campaign_panel.hpp"
#include "../view/sfml_view.hpp"
#include "../view/tech_panel.hpp"
#include <SFML/Window/Keyboard.hpp>
#include <sys/stat.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <string>

static bool keyEdge(sf::Keyboard::Key k) {
  static bool s[sf::Keyboard::KeyCount] = {false};
  if (sf::Keyboard::isKeyPressed(k)) {
    if (!s[k]) {
      s[k] = true;
      return true;
    }
    return false;
  }
  s[k] = false;
  return false;
}

int main() {
  using namespace sim;
  play::PlaygroundSim sim(42);
  sim.setMode(play::Mode::Pid);

  SfmlView view;
  TechPanel tech;
  tech.init();
  sf::Clock clk, fps_clk;
  double acc = 0;
  const double DT = play::PlaygroundSim::DT;
  int frames = 0;
  double fps = 60;
  char title[256];
  bool follow_cam = false, show_trail = true, show_lidar = true;
  std::vector<Vec2> trail;
  auto patrols = demoPatrols();
  Vec2 goal = mazeGoal();

  // ---- M5战役: 选关/结算逻辑在camp::Play, 这里只管存档+后台跑+面板 ----
  camp::Play play;
  std::string save_path = "./campaign.save";
  if (const char* home = std::getenv("HOME")) {
    std::string dir = std::string(home) + "/.local/share/robot_simu";
    mkdir((std::string(home) + "/.local/share").c_str(), 0755);
    mkdir(dir.c_str(), 0755);
    save_path = dir + "/campaign.save";
  }
  if (FILE* f = fopen(save_path.c_str(), "r")) {
    unsigned m = 0;
    if (fscanf(f, "%u", &m) == 1) play.state().loadMask((uint32_t)m);
    fclose(f);
  }
  auto save = [&]() {
    if (FILE* f = fopen(save_path.c_str(), "w")) {
      fprintf(f, "%u\n", play.state().mask());
      fclose(f);
    }
  };
  CampaignPanel cpanel;
  cpanel.init();
  cpanel.setVisible(true);  // 开机即显: 战役是主线, 不能藏在N后面等玩家发现
  std::future<task::Metrics> run_fut;
  int run_task_id = 0;
  bool running = false;
  std::string toast;
  int toast_frames = 0;

  while (!view.pollClose()) {
    if (keyEdge(sf::Keyboard::Num0)) sim.setMode(play::Mode::Teleop);
    if (keyEdge(sf::Keyboard::Num1)) sim.setMode(play::Mode::Pid);
    if (keyEdge(sf::Keyboard::Num2)) sim.setMode(play::Mode::Astar);
    if (keyEdge(sf::Keyboard::Num3)) sim.setMode(play::Mode::Dwa);
    if (keyEdge(sf::Keyboard::Num4)) sim.setMode(play::Mode::Odom);
    if (keyEdge(sf::Keyboard::F)) tech.toggle();
    if (keyEdge(sf::Keyboard::O)) sim.setFog(!sim.fogOn());  // 曾用G, 与DWA调参撞键
    if (keyEdge(sf::Keyboard::V)) follow_cam = !follow_cam;
    if (keyEdge(sf::Keyboard::P)) show_trail = !show_trail;  // 曾用T, 与DWA调参撞键
    if (keyEdge(sf::Keyboard::L)) show_lidar = !show_lidar;
    if (keyEdge(sf::Keyboard::C)) {
      double s = sim.slip();
      sim.setSlip(s == 0 ? 1.0 : (s == 1.0 ? 3.0 : 0.0));
    }
    if (keyEdge(sf::Keyboard::K)) sim.kidnap(3.0, 2.0);  // 真值跳变,odom不知
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
      sim.setPid(sim.pid().kp() + 0.004, sim.pid().ki(), sim.pid().kd());
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
      sim.setPid(sim.pid().kp() - 0.004, sim.pid().ki(), sim.pid().kd());
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
      sim.setPid(sim.pid().kp(), sim.pid().ki() + 0.002, sim.pid().kd());
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
      sim.setPid(sim.pid().kp(), sim.pid().ki() - 0.002, sim.pid().kd());
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::E))
      sim.setPid(sim.pid().kp(), sim.pid().ki(), sim.pid().kd() + 0.001);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
      sim.setPid(sim.pid().kp(), sim.pid().ki(), sim.pid().kd() - 0.001);
    if (keyEdge(sf::Keyboard::Z)) sim.setHw(sim.hw() + 0.5);
    if (keyEdge(sf::Keyboard::X)) sim.setHw(sim.hw() - 0.5);
    if (keyEdge(sf::Keyboard::T)) sim.tuneDwa(0.2, 0, 0);
    if (keyEdge(sf::Keyboard::G)) sim.tuneDwa(-0.2, 0, 0);
    if (keyEdge(sf::Keyboard::Y)) sim.tuneDwa(0, 0.2, 0);
    if (keyEdge(sf::Keyboard::H)) sim.tuneDwa(0, -0.2, 0);
    if (keyEdge(sf::Keyboard::U)) sim.tuneDwa(0, 0, 0.1);
    if (keyEdge(sf::Keyboard::J)) sim.tuneDwa(0, 0, -0.1);
    if (keyEdge(sf::Keyboard::R)) {
      sim.resetAll();
      trail.clear();
    }
    if (keyEdge(sf::Keyboard::N)) cpanel.toggle();
    if (keyEdge(sf::Keyboard::LBracket)) play.selectPrev();
    if (keyEdge(sf::Keyboard::RBracket)) play.selectNext();
    if (keyEdge(sf::Keyboard::Enter) && !running && play.canRunSelected()) {
      running = true;
      run_task_id = play.selected();
      task::TaskConfig cfg{0};
      for (auto& t : task::allTasks())
        if (t.id == run_task_id) cfg = t;
      // 玩家亲手调的PID/DWA/h_w叠上去: 默认值=答案(行为不变), 调废真失败
      task::Loadout lo =
          camp::mergePlayerTuning(camp::suggestedLoadout(run_task_id), sim.pid(),
                                  sim.dwaWeights(), sim.hw());
      run_fut = std::async(std::launch::async, [cfg, lo]() { return task::run_task(cfg, lo); });
    }
    // B反面演示: 跑本关的必败配置, 看失败+诊断(同样走结算, 失败数+1线索升级)
    if (keyEdge(sf::Keyboard::B) && !running && play.canRunSelected()) {
      running = true;
      run_task_id = play.selected();
      task::TaskConfig cfg{0};
      for (auto& t : task::allTasks())
        if (t.id == run_task_id) cfg = t;
      task::Loadout lo = camp::badLoadout(run_task_id);
      run_fut = std::async(std::launch::async, [cfg, lo]() { return task::run_task(cfg, lo); });
    }
    // 后台任务结算(非阻塞轮询): 成功解锁+存档, 失败线索升级
    if (running && run_fut.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      task::Metrics m = run_fut.get();
      running = false;
      const camp::Level* l = camp::findLevel(run_task_id);
      char b[1024];
      if (m.success) {
        play.applyResult(run_task_id, true);
        save();
        snprintf(b, sizeof b, "L%d %s 完成! %s 卡:[%s]%s — %s", run_task_id,
                 l ? l->name : "", l ? l->story_after : "", l ? l->card.title : "",
                 l ? l->card.id : "", l ? l->card.body : "");
      } else {
        play.applyResult(run_task_id, false, m.fail_reason);
        snprintf(b, sizeof b, "L%d 失败:%s. 新线索:%s 诊断:%s", run_task_id, m.fail_reason,
                 camp::clueFor(run_task_id, play.fails(run_task_id)),
                 camp::hintFor(m.fail_reason));
      }
      toast = b;
      toast_frames = 900;
    }
    if (toast_frames > 0) toast_frames--;

    acc += clk.restart().asSeconds();
    int n = 0;
    while (acc >= DT && n < 20) {
      acc -= DT;
      n++;
      play::TeleopInput in;
      in.up = sf::Keyboard::isKeyPressed(sf::Keyboard::Up);
      in.down = sf::Keyboard::isKeyPressed(sf::Keyboard::Down);
      in.left = sf::Keyboard::isKeyPressed(sf::Keyboard::Left);
      in.right = sf::Keyboard::isKeyPressed(sf::Keyboard::Right);
      sim.step(in, 1);
    }
    frames++;
    if (fps_clk.getElapsedTime().asSeconds() > 0.5) {
      fps = frames / fps_clk.restart().asSeconds();
      frames = 0;
    }

    double rx, ry, rth;
    sim.activePose(rx, ry, rth);
    trail.push_back({rx, ry});
    if (trail.size() > 4000) trail.erase(trail.begin(), trail.begin() + 500);

    std::vector<DynState> dyn;
    for (auto& p : patrols) dyn.push_back(patrol_state(p, sim.simTime()));

    view.follow(follow_cam, rx, ry);
    view.begin();
    view.drawMap(sim.truthMap());
    if (sim.fogOn()) view.drawFog(sim.fog());
    view.drawPath(sim.astar().path);
    view.drawGoal(goal);
    if (show_trail) view.drawTrail(trail);
    if (show_lidar) view.drawLidar({rx, ry}, rth, sim.scan(), 3);
    if (sim.mode() == play::Mode::Dwa) {
      for (auto& s : dyn) view.drawFork(s.p, s.radius);
      view.drawRollouts(sim.lastDwa().rollouts);
      view.drawRobot(rx, ry, rth);
      view.drawGhost(sim.odom().p.x, sim.odom().p.y, sim.odom().th);
      tech.pushPid(0, 0, 0, 0);  // PID不参与DWA模式, 曲线归零不断章取义
      tech.setDwa(sim.lastDwa().best, sim.dwaWeights(), sim.lastDwa().rollouts.size());
      double gap = 99;
      for (auto& s : dyn)
        gap = std::min(gap, std::hypot(rx - s.p.x, ry - s.p.y) - s.radius - kRobotRadius);
      tech.setDyn(gap);
      snprintf(title, sizeof title, "M4-DWA v=%.2f w=%.2f gap=%.2f drift=%.2f | T/G Y/H U/J tune",
               sim.dwaV(), sim.dwaW(), gap, sim.odom().driftErr({rx, ry}));
    } else if (sim.mode() == play::Mode::Teleop) {
      tech.setDyn(99);
      view.drawRobot(rx, ry, rth);
      view.drawGhost(sim.odom().p.x, sim.odom().p.y, sim.odom().th);
      tech.pushPid(0, 0, 0, 0);  // teleop无PID, 曲线归零
      tech.setDwa({0, 0, false, 0}, sim.dwaWeights(), 0);
      snprintf(title, sizeof title, "M4-teleop v=%.2f w=%.2f drift=%.2f | arrows drive, C slip",
               sim.teleop().v, sim.teleop().w, sim.odom().driftErr({rx, ry}));
    } else {
      tech.setDyn(99);
      view.drawRobot(rx, ry, rth);
      view.drawGhost(sim.odom().p.x, sim.odom().p.y, sim.odom().th);
      tech.setDwa({0, 0, false, 0}, sim.dwaWeights(), 0);
      tech.pushPid(sim.lastErr(), sim.lastP(), sim.lastI(), sim.lastD());
      snprintf(title, sizeof title, "M4 mode=%d Kp=%.2f Ki=%.2f Kd=%.3f h=%.1f wp=%zu/%zu drift=%.2f | F",
               (int)sim.mode(), sim.pid().kp(), sim.pid().ki(), sim.pid().kd(), sim.hw(),
               sim.wpIdx(), sim.wpTotal(), sim.odom().driftErr({rx, ry}));
    }
    tech.setAstar(sim.hw(), sim.astar().expanded, sim.astar().path.size());
    tech.setPidGains(sim.pid().kp(), sim.pid().ki(), sim.pid().kd());
    tech.setStatus((int)sim.mode(), fps);
    tech.setLoc(sim.odom().driftErr({rx, ry}), sim.lidarNoise(), sim.slip(), sim.scan().n_rays);
    tech.setEvents(sim.events().events());
    // 战役面板只读快照(逻辑全在camp::Play)
    if (const camp::Level* l = camp::findLevel(play.selected())) {
      cpanel.setDetail(l->story_before, l->brief, play.clueSelected(),
                       play.fails(play.selected()));
      cpanel.setLastFail(play.failReason(play.selected()), play.hintSelected());
    }
    cpanel.setLevels(camp::levels(), play.state(), play.selected());
    char rl[32];
    snprintf(rl, sizeof rl, "L%d", run_task_id);
    cpanel.setRunning(running, rl);
    cpanel.setToast(toast_frames > 0 ? toast : "");
    view.beginOverlays();
    tech.draw(view.window());
    cpanel.draw(view.window());
    view.end(title);
  }
  return 0;
}
