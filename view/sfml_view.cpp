#include "sfml_view.hpp"
#include "../core/fogmap.hpp"
#include <cmath>
SfmlView::SfmlView(int px_w,int px_h): win_(sf::VideoMode(px_w,px_h),"robot_simu M4") { win_.setFramerateLimit(60); }
bool SfmlView::pollClose(){ sf::Event e; while(win_.pollEvent(e)) if(e.type==sf::Event::Closed) return true; return false; }
void SfmlView::begin(){ win_.clear(sf::Color(18,18,24));
  if(follow_){ sf::View v=win_.getDefaultView(); v.setCenter(toPx(fx_,fy_)); win_.setView(v); }
  else win_.setView(win_.getDefaultView());
}
void SfmlView::follow(bool on,double x,double y){ follow_=on; fx_=x; fy_=y; }
void SfmlView::beginOverlays(){ win_.setView(win_.getDefaultView()); }
sf::Vector2f SfmlView::toPx(double wx,double wy){ return { (float)(wx*scale_+20), (float)(win_.getSize().y-20-wy*scale_) }; }
void SfmlView::drawMap(const sim::GridMap& m){
  sf::RectangleShape cell(sf::Vector2f((float)(m.res()*scale_-1),(float)(m.res()*scale_-1)));
  cell.setFillColor(sf::Color(90,90,110));
  for(int gy=0;gy<m.h();gy++) for(int gx=0;gx<m.w();gx++)
    if(!m.isFreeCell(gx,gy)){ auto p=m.gridToWorld(gx,gy); cell.setPosition(toPx(p.x-m.res()/2,p.y+m.res()/2)); win_.draw(cell); }
}
void SfmlView::drawPath(const std::vector<sim::Vec2>& path){
  if(path.size()<2) return;
  sf::VertexArray l(sf::LineStrip, path.size());
  for(size_t i=0;i<path.size();i++){ l[i].position=toPx(path[i].x,path[i].y); l[i].color=sf::Color::Green; }
  win_.draw(l);
}
void SfmlView::drawRobot(double x,double y,double th){
  sf::ConvexShape tri(3);
  tri.setPoint(0,sf::Vector2f(14,0)); tri.setPoint(1,sf::Vector2f(-10,9)); tri.setPoint(2,sf::Vector2f(-10,-9));
  tri.setFillColor(sf::Color::Cyan);
  auto p=toPx(x,y); tri.setPosition(p); tri.setRotation((float)(-th*180/3.14159));
  win_.draw(tri);
}
void SfmlView::drawRollouts(const std::vector<std::pair<sim::Vec2,double>>& r){
  sf::CircleShape c(2); c.setFillColor(sf::Color(255,180,0,120));
  for(auto &e:r){ c.setPosition(toPx(e.first.x,e.first.y)); win_.draw(c); }
}
void SfmlView::drawTrail(const std::vector<sim::Vec2>& tr){
  if(tr.size()<2) return;
  sf::VertexArray l(sf::LineStrip, tr.size());
  for(size_t i=0;i<tr.size();i++){ l[i].position=toPx(tr[i].x,tr[i].y); l[i].color=sf::Color(80,160,255,200); }
  win_.draw(l);
}
void SfmlView::drawLidar(sim::Vec2 pos, double th, const sim::Scan& s, int stride){
  auto p0=toPx(pos.x,pos.y);
  for(int i=0;i<s.n_rays;i+=stride){
    double a=th+2*M_PI*i/s.n_rays;
    double r=s.ranges[i];
    sf::Vector2f p1=toPx(pos.x+std::cos(a)*r, pos.y+std::sin(a)*r);
    sf::Vertex ln[]={{p0,sf::Color(60,200,200,90)},{p1,sf::Color(60,200,200,90)}};
    win_.draw(ln,2,sf::Lines);
    sf::CircleShape c(1.5f); c.setFillColor(sf::Color(100,230,230,160)); c.setPosition(p1); win_.draw(c);
  }
}
void SfmlView::drawGhost(double x,double y,double th){
  sf::ConvexShape tri(3);
  tri.setPoint(0,sf::Vector2f(14,0)); tri.setPoint(1,sf::Vector2f(-10,9)); tri.setPoint(2,sf::Vector2f(-10,-9));
  tri.setFillColor(sf::Color::Transparent);
  tri.setOutlineColor(sf::Color::Red); tri.setOutlineThickness(2);
  auto p=toPx(x,y); tri.setPosition(p); tri.setRotation((float)(-th*180/3.14159));
  win_.draw(tri);
}
void SfmlView::drawGoal(sim::Vec2 g){  sf::CircleShape c(7); c.setFillColor(sf::Color::Transparent);
  c.setOutlineColor(sf::Color::Magenta); c.setOutlineThickness(2);
  auto p=toPx(g.x,g.y); c.setPosition(p.x-7,p.y-7); win_.draw(c);
}
void SfmlView::drawFork(sim::Vec2 p, double r){
  float pr=(float)(r*scale_);
  sf::CircleShape c(pr); c.setFillColor(sf::Color(255,140,0,200));
  auto q=toPx(p.x,p.y); c.setPosition(q.x-pr,q.y-pr); win_.draw(c);
}
void SfmlView::drawFog(const sim::FogMap& f){
  sf::RectangleShape cell(sf::Vector2f((float)(f.res()*scale_),(float)(f.res()*scale_)));
  cell.setFillColor(sf::Color(8,8,12,210));
  for(int gy=0;gy<f.h();gy++) for(int gx=0;gx<f.w();gx++){
    double cx=(gx+0.5)*f.res(), cy=(gy+0.5)*f.res();
    if(!f.seenAt(cx,cy)){ cell.setPosition(toPx(cx-f.res()/2,cy+f.res()/2)); win_.draw(cell); }
  }
}
void SfmlView::end(const char* title){ win_.setTitle(title); win_.display(); }
