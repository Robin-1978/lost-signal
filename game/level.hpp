#pragma once
#include "../core/grid_map.hpp"
#include "../core/dynobs.hpp"
#include <vector>
// S形迷宫: 两堵错开的墙,逼出长绕路.h_w=1最优但扩展多,h_w=3贪心扩展少但路径长.
// wall1 x8-9 y0-11(顶部留3m缺口), wall2 x12-13 y3-14(底部留3m缺口).
inline sim::GridMap buildMaze(){
  sim::GridMap m(40,28,0.5); // 20m x 14m
  m.addBox(8,0,9,11);
  m.addBox(12,3,13,14);
  return m;
}
inline sim::Vec2 mazeStart(){ return {1,1}; }
inline sim::Vec2 mazeGoal(){ return {17,10}; }
// t6巡逻叉车: 纵切底部缺口(x=12.5), 机器人横穿时相位相遇;
// 有DWA减速/让行可过, 致盲直穿必撞(时序由种子固定).
inline std::vector<sim::Patrol> demoPatrols(){
  return { {{12.5,0.5},{12.5,4.5},0.39,0.45} };
}
