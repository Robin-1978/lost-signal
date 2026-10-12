#include "tech_panel.hpp"
#include <cstdio>
bool TechPanel::init(){
  has_font_ = font_.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
  return has_font_;
}
void TechPanel::pushPid(double err,double p,double i,double d){
  err_hist_.push_back((float)err);
  if(err_hist_.size()>CAP) err_hist_.erase(err_hist_.begin());
  cur_p_=p; cur_i_=i; cur_d_=d;
}
void TechPanel::setAstar(double h,int e,size_t l){ h_=h; expanded_=e; path_len_=l; }
void TechPanel::setDwa(sim::DwaCmd b, sim::DwaWeights w, size_t n){ best_=b; weights_=w; nroll_=n; }
void TechPanel::setStatus(int m,double fps){ mode_=m; fps_=fps; }
void TechPanel::text(sf::RenderWindow& win,const std::string& s,float x,float y,int size,sf::Color c){
  if(!has_font_) return;
  sf::Text t(s,font_,(unsigned)size); t.setFillColor(c); t.setPosition(x,y); win.draw(t);
}
void TechPanel::drawCurve(sf::RenderWindow& win,float x0,float y0,float w,float h){
  sf::RectangleShape bg(sf::Vector2f(w,h));
  bg.setPosition(x0,y0); bg.setFillColor(sf::Color(10,10,16,220)); bg.setOutlineColor(sf::Color(80,80,100));
  bg.setOutlineThickness(1); win.draw(bg);
  // zero line
  sf::Vertex z[]={{{x0,y0+h/2},sf::Color(120,120,120)},{{x0+w,y0+h/2},sf::Color(120,120,120)}};
  win.draw(z,2,sf::Lines);
  if(err_hist_.size()<2) return;
  sf::VertexArray line(sf::LineStrip,err_hist_.size());
  for(size_t i=0;i<err_hist_.size();i++){
    float px=x0+w*(float)i/(CAP-1);
    float e=err_hist_[i]; if(e>1)e=1; if(e<-1)e=-1;
    float py=y0+h/2-e*(h/2-2);
    line[i].position={px,py}; line[i].color=sf::Color::Cyan;
  }
  win.draw(line);
}
void TechPanel::draw(sf::RenderWindow& win){
  if(!visible_) return;
  float px0=(float)win.getSize().x-300.f;
  sf::RectangleShape panel(sf::Vector2f(300,(float)win.getSize().y));
  panel.setPosition(px0,0); panel.setFillColor(sf::Color(22,22,30,235)); win.draw(panel);
  char b[256];
  text(win,"TECH PANEL (F hide)",px0+10,8,13,sf::Color::Yellow);
  snprintf(b,sizeof b,"mode=%d fps=%.0f",mode_,fps_);
  text(win,b,px0+10,30,12,sf::Color(180,180,180));
  // PID
  text(win,"-- PID heading --",px0+10,54,12,sf::Color::Green);
  snprintf(b,sizeof b,"Kp=%.3f Ki=%.3f Kd=%.3f",kp_,ki_,kd_);
  text(win,b,px0+10,72,12);
  snprintf(b,sizeof b,"P=%.3f I=%.3f D=%.3f",cur_p_,cur_i_,cur_d_);
  text(win,b,px0+10,90,12);
  text(win,"err(rad) hist 8s, Q/A W/S E/D",px0+10,108,11,sf::Color(150,150,150));
  drawCurve(win,px0+10,126,280,90);
  // A*
  text(win,"-- A* global --",px0+10,226,12,sf::Color::Green);
  snprintf(b,sizeof b,"h_w=%.1f expanded=%d len=%zu",h_,expanded_,path_len_);
  text(win,b,px0+10,244,12);
  text(win,"Z/X h_w + auto replan",px0+10,262,11,sf::Color(150,150,150));
  // DWA
  text(win,"-- DWA local --",px0+10,286,12,sf::Color::Green);
  snprintf(b,sizeof b,"wg=%.1f wc=%.1f ws=%.1f",weights_.w_goal,weights_.w_clear,weights_.w_speed);
  text(win,b,px0+10,304,12);
  if(best_.valid) snprintf(b,sizeof b,"best v=%.2f w=%.2f s=%.2f n=%zu",best_.v,best_.w,best_.score,nroll_);
  else snprintf(b,sizeof b,"no valid cmd n=%zu",nroll_);
  text(win,b,px0+10,322,12);
  text(win,"T/G wg  Y/H wc  U/J ws",px0+10,340,11,sf::Color(150,150,150));
  text(win,"0 tele  1 PID  2 Astar  3 DWA  4 Odom  R reset",px0+10,364,11,sf::Color(255,200,100));
  text(win,"L lidar C slip K kidnap O fog P trail",px0+10,380,11,sf::Color(255,200,100));
  // Loc
  text(win,"-- percept/loc --",px0+10,404,12,sf::Color::Green);
  snprintf(b,sizeof b,"drift=%.2fm slip=x%.0f nz=%.2f n=%d",drift_,slip_,lidar_noise_,nrays_);
  text(win,b,px0+10,422,12, drift_>1.0?sf::Color::Red:sf::Color::White);
  text(win,"cyan=true red=odom",px0+10,440,11,sf::Color(150,150,150));
  snprintf(b,sizeof b,"dyn nearest=%.2fm",dyn_nearest_);
  text(win,b,px0+10,456,12, dyn_nearest_<0.8?sf::Color::Red:sf::Color::White);
  text(win,"tradeoff:",px0+10,482,11,sf::Color::Yellow);
  text(win,"h_w up: faster, longer path",px0+10,498,11);
  text(win,"wc up: safer, slower/shy",px0+10,514,11);
  text(win,"Kd up: less overshoot,",px0+10,530,11);
  text(win,"  too big: oscillation",px0+10,546,11);
  // Events
  text(win,"-- events --",px0+10,570,12,sf::Color::Green);
  size_t e0=ev_show_.size()>8?ev_show_.size()-8:0;
  int row=0;
  for(size_t i=e0;i<ev_show_.size();i++,row++){
    snprintf(b,sizeof b,"%.1fs %s %.2f",ev_show_[i].t,sim::ev_name(ev_show_[i].code),ev_show_[i].aux);
    text(win,b,px0+10,588+row*15,11,sf::Color(255,210,150));
  }
}
