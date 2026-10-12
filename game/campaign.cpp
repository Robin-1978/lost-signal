#include "campaign.hpp"

namespace camp {
namespace {
const std::vector<Level> kLevels = {
  {1, "醒来",
   "你在一间工作间里醒来, 轮子还能转, 但地图一片空白. 先开出去再说.",
   "门开了. 原来直线也能走偏——执行器是有极限的.",
   "目标: 从(1,1)到达(8,6). 约束: 零碰撞.",
   "先试着直线开过去, 观察速度是否跟得上指令.",
   "底盘加速度/角加速度有限幅, 大指令会被钳住, 不是瞬时到达.",
   "teleop按Up给tv=1.0即可; 若用代码, tv别超过v_max=2.0, 留好刹车距离.",
   {"pid-vmax", "执行器限幅", "速度/转向指令都会被饱和与变化率钳住, 看到跟不上指令是正常的, 不是bug."}},
  {2, "第一只眼",
   "仓库管理员给你装了一只激光眼. 先学会在撞墙前停下.",
   "你第一次'看见'了墙. 没lidar的你, 只会一头撞上去.",
   "目标: 沿初始朝向直行, 在障碍2m内停下. 瞎子loadout必撞.",
   "盯着正前方±0.35rad的锥形区, 看最小回波距离.",
   "lidar按36线、observe_range=2.0做检测; 连续3拍(10Hz)确认才算看见.",
   "用has_lidar=true、lidar_range=8.0、observe_range=2.0; 无lidar对比跑一次看collision-blind.",
   {"lidar-cone", "激光检测锥", "只信正前方小锥角内的最小range, 连续多拍确认再停车, 否则噪声会骗你急刹."}},
  {3, "撞墙之谜",
   "8米直线, 你明明照着地图开, 却一次次蹭墙. 地图没错, 错的是你以为的位置.",
   "里程计在撒谎: 滑移噪声×距离=漂移. 这就是 crashes 的真相.",
   "目标: 用估计位姿跟随走完8米直线. slip越大漂移越大.",
   "对比slip=0和slip=3跑同一条线, 看终点漂移差多少.",
   "DeadReckon用带噪v/w积分, yaw_bias=0.02*slip是盖不住的系统性漂移源.",
   "loadout: slip_scale=0干净 / 1正常 / 3很差; drift>1m即报EV_DRIFT_HIGH.",
   {"odom-drift", "里程计漂移", "估计位姿=带噪积分, 误差随路程累积; 长走必偏, 漂移超1m就该怀疑定位, 而不是规划."}},
  {4, "地图",
   "S形迷宫. 有人把墙 Priority 到处乱放, 你的轮廓必须先'变胖'.",
   "膨胀层救了你: 给车体留出净空, 宁可绕远, 不蹭墙.",
   "目标: S迷宫从(1,1)到(17,10), 零碰撞. 用膨胀地图规划.",
   "先用inflation=0.4跑一次, 再试0.0看蹭墙率.",
   "costmap=真值膨胀0.4m, A*在膨胀层上规划, 跟随用LOS检查保净空.",
   "loadout: inflation=0.4, h_weight=1.5, cruise=1.0, Ld=2.0; 起点若陷进膨胀格会被snapFree捞出.",
   {"inflation", "膨胀层", "规划前先把障碍按机器人半径+余量膨胀; 路径变长, 但车体不再蹭墙, 这是拿路程换安全."}},
  {5, "绕行",
   "同一座迷宫, 两种走法: 最优但想得久, 贪心但走得远. 你选哪个?",
   "h=1最优、扩展多; h=3贪心、扩展少但路径长. 这就是第一个真正的取舍.",
   "目标: h_w=1和h_w=3都要能到, 对比expanded与path_len.",
   "两种h_w各跑一次, 记录expanded和路径点数.",
   "8向A*, octile启发×h_weight; h=1最优, >1次优但快, S迷宫里扩展614 vs 362.",
   "playground按Z/X切h_w会重规划; task t5要求两种配置都成功.",
   {"astar-h", "A*启发权重", "h_weight=1保证最优, 调大搜索快但路径变长; 看expanded与path_len的此消彼长再选."}},
  {6, "运输",
   "能源室缺货, 得先经停补给点(10.5,12.5), 还要躲开巡逻叉车. 闭眼直穿必被撞.",
   "DWA让你活了下来: 全局指方向, 局部管刹车, 叉车外推0.8m内算惊险会车.",
   "目标: 经停via再到终点, 零碰撞(含动障碍). 开DWA vs 致盲直穿对比.",
   "先跑use_dwa=false看collision-dyn, 再开DWA看让行.",
   "前视点只给局部目标, 速度由DWA定时变打分(含巡逻车线性外推), 无可行指令就地停车.",
   "loadout: use_dwa=true, dwa={w_goal=1,w_clear=2,w_speed=0.3}, v_cap按朝向误差调度, 保底0.25.",
   {"dwa-dyn", "DWA避障", "w_goal冲目标 / w_clear保安全 / w_speed求快三者打架; 动障碍用位置+速度外推, 没解就停, 别硬穿."}},
  {7, "迷雾",
   "最后一间仓库, 地图未知. 乐观者敢闯, 保守者寸步难行, 只有边扫边重规划能出去.",
   "雾散了: 10Hz揭示+周期重规划, 从估计位姿出发, 乐观未知=空闲.",
   "目标: fog+replan闭环走出迷宫. 对比: 开环必撞, 保守(未知当墙)无路.",
   "先跑fog=false看开环撞墙, 再跑unknown_free=false看no-path, 最后跑完整闭环.",
   "lidar只信hit标志建图, 规划格=已见格+未知格(乐观空闲/保守占位), 按replan_interval步重规划.",
   "loadout: fog=true, unknown_free=true, replan=true, replan_interval=100步(1s).",
   {"fog-replan", "雾战重规划", "机器人只知道自己扫过的格子; 乐观敢闯+周期重规划形成感知-规划闭环, 保守当墙则经常无解."}},
};
}  // namespace

const std::vector<Level>& levels() { return kLevels; }

const Level* findLevel(int task) {
  for (auto& l : kLevels)
    if (l.task == task) return &l;
  return nullptr;
}

const char* clueFor(int task, int fails) {
  const Level* l = findLevel(task);
  if (!l) return "";
  if (fails < 0) fails = 0;
  if (fails == 0) return l->clue1;
  if (fails == 1) return l->clue2;
  return l->clue3;
}

void CampaignState::reset() { done_ = 0; }
size_t CampaignState::total() const { return kLevels.size(); }

bool CampaignState::unlocked(int task) const {
  if (!findLevel(task)) return false;
  if (task == kLevels.front().task) return true;
  // 前一关完成即解锁
  for (size_t i = 1; i < kLevels.size(); i++)
    if (kLevels[i].task == task) return completed(kLevels[i - 1].task);
  return false;
}

bool CampaignState::completed(int task) const {
  if (!findLevel(task) || task < 1 || task > 32) return false;
  return (done_ & (1u << (task - 1))) != 0;
}

bool CampaignState::complete(int task, bool success) {
  if (!success || !unlocked(task) || completed(task)) return false;
  done_ |= (1u << (task - 1));
  return true;
}

std::vector<Card> CampaignState::cards() const {
  std::vector<Card> out;
  for (auto& l : kLevels)
    if (completed(l.task)) out.push_back(l.card);
  return out;
}

int CampaignState::progress() const {
  int n = 0;
  for (auto& l : kLevels)
    if (completed(l.task)) n++;
  return n;
}

uint32_t CampaignState::mask() const { return done_; }

bool CampaignState::loadMask(uint32_t m) {
  uint32_t allow = 0;
  for (auto& l : kLevels) allow |= (1u << (l.task - 1));
  if (m & ~allow) return false;
  // 读档必须保持前缀闭合: 不能跳过未完成关卡解锁后面
  for (size_t i = 1; i < kLevels.size(); i++) {
    bool cur = (m & (1u << (kLevels[i].task - 1))) != 0;
    bool prev = (m & (1u << (kLevels[i - 1].task - 1))) != 0;
    if (cur && !prev) return false;
  }
  done_ = m;
  return true;
}
}  // namespace camp
