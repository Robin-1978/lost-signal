#pragma once
#include <SFML/Graphics.hpp>
#include "../core/grid_map.hpp"
#include "../core/math.hpp"
#include "../core/sensor.hpp"
#include "../core/fogmap.hpp"
#include <vector>
// View只做画: 不含任何规划/动力学, 从Core拿只读状态.
class SfmlView {
public:
  SfmlView(int px_w=960,int px_h=720);
  bool pollClose();
  void begin();
  void drawMap(const sim::GridMap& m);
  void drawPath(const std::vector<sim::Vec2>& path);
  void drawRobot(double x,double y,double th);
  void drawRollouts(const std::vector<std::pair<sim::Vec2,double>>& r);
  void drawLidar(sim::Vec2 pos, double th, const sim::Scan& s, int stride=4);
  void drawGhost(double x,double y,double th);
  void drawGoal(sim::Vec2 g);
  void drawFork(sim::Vec2 p, double r); // 动障碍: 橙色圆+半径
  void drawFog(const sim::FogMap& f); // 未知格压暗(机器人视角)
  void drawTrail(const std::vector<sim::Vec2>& tr); // 实际轨迹层
  void follow(bool on, double x=0, double y=0); // 机器人跟随相机(V)
  void beginOverlays(); // 覆盖层(面板)切回默认视图
  void end(const char* title);
  sf::RenderWindow& window(){ return win_; }
  float scale() const { return scale_; }
private:
  sf::RenderWindow win_;
  float scale_=30.f; // px per meter
  bool follow_{false}; double fx_{0}, fy_{0};
  sf::Vector2f toPx(double wx,double wy);
};
