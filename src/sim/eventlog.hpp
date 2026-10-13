#pragma once
#include <vector>
#include <cstdint>
// 事件日志(Core, 无图形): 失败诊断的数据地基.
// runner和app playground共用; TechPanel只读展示. 时间+编码+辅助值, 同码节流.
namespace sim {
enum EvCode : uint8_t {
  EV_COLLISION=0, // 撞静态墙(含瞎走)
  EV_DYN_HIT,     // 撞动障碍
  EV_DYN_NEAR,    // 与动障碍净距离<0.8(惊险会车)
  EV_DRIFT_HIGH,  // odom漂移>1m
  EV_DWA_STOP,    // DWA无可行指令, 原地停车
  EV_VIA,         // 经过经停点
  EV_REACHED,     // 到达目标
  EV_NO_PATH,     // A*无解
  EV_LOST,        // 走完未到(漂移跟丢等)
  EV_COUNT
};
inline const char* ev_name(uint8_t c){
  static const char* n[EV_COUNT]={"COLLISION","DYN_HIT","DYN_NEAR","DRIFT_HIGH","DWA_STOP","VIA","REACHED","NO_PATH","LOST"};
  return c<EV_COUNT?n[c]:"?";
}
struct Event { double t{0}; uint8_t code{0}; float aux{0}; };
class EventLog {
public:
  void clear(){ ev_.clear(); for(auto &x:last_)x=-1e9; }
  // 同码min_gap秒内只记一次(防刷屏), 返回是否记入
  bool log(double t, uint8_t code, float aux=0, double min_gap=2.0){
    if(code>=EV_COUNT) return false;
    if(t-last_[code]<min_gap) return false;
    last_[code]=t; ev_.push_back({t,code,aux});
    return true;
  }
  const std::vector<Event>& events() const { return ev_; }
  uint32_t mask() const { uint32_t m=0; for(auto &e:ev_) m|=(1u<<e.code); return m; }
private:
  std::vector<Event> ev_;
  double last_[EV_COUNT];
};
}
