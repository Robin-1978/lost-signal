#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include "../core/dwa.hpp"
#include "../core/eventlog.hpp"
// View-only: 只读Core状态做可视化,不含任何算法.
// 历史曲线 + A*/DWA统计,给玩家看取舍.
class TechPanel {
public:
  bool init();
  void setVisible(bool v){ visible_=v; }
  bool visible() const { return visible_; }
  void toggle(){ visible_=!visible_; }
  void pushPid(double err,double p,double i,double d);
  void setAstar(double h,int expanded,size_t len);
  void setDwa(sim::DwaCmd best, sim::DwaWeights w, size_t nroll);
  void setLoc(double drift, double lidar_noise, double slip, int nrays){ drift_=drift; lidar_noise_=lidar_noise; slip_=slip; nrays_=nrays; }
  void setDyn(double nearest){ dyn_nearest_=nearest; }
  void setEvents(const std::vector<sim::Event>& ev){
    ev_show_.clear();
    size_t n=ev.size()>24?24:ev.size(); // 只留最近24条
    for(size_t i=ev.size()-n;i<ev.size();i++) ev_show_.push_back(ev[i]);
  }
  void setStatus(int mode,double fps);
  void setPidGains(double kp,double ki,double kd){ kp_=kp;ki_=ki;kd_=kd; }
  void draw(sf::RenderWindow& win);
  // headless可测的逻辑
  size_t histSize() const { return err_hist_.size(); }
  double lastErr() const { return err_hist_.empty()?0:err_hist_.back(); }
private:
  bool visible_=true;
  sf::Font font_; bool has_font_=false;
  std::vector<float> err_hist_; static constexpr size_t CAP=400;
  double cur_p_{0},cur_i_{0},cur_d_{0};
  double kp_{0},ki_{0},kd_{0};
  double h_{1.5}; int expanded_{0}; size_t path_len_{0};
  sim::DwaCmd best_; sim::DwaWeights weights_; size_t nroll_{0};
  double drift_{0}, lidar_noise_{0.05}, slip_{1.0}; int nrays_{72};
  double dyn_nearest_{99}; // 最近动障碍净距离(无动障碍=99)
  std::vector<sim::Event> ev_show_;
  int mode_{1}; double fps_{60};
  void drawCurve(sf::RenderWindow& win,float x0,float y0,float w,float h);
  void text(sf::RenderWindow& win,const std::string& s,float x,float y,int size=12,sf::Color c=sf::Color::White);
};
