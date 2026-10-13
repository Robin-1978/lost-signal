#include "astar.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <limits>
namespace sim {
AStarResult astar_plan(const GridMap& cm, Vec2 start, Vec2 goal, double h_weight) {
 AStarResult r;
 auto [sx, sy] = cm.worldToGrid(start.x, start.y);
 auto [gx, gy] = cm.worldToGrid(goal.x, goal.y);
 int W = cm.w(), H = cm.h();
 auto idx = [&](int x, int y) { return y * W + x; };
 if (!cm.inside(sx, sy) || !cm.inside(gx, gy))
  return r;
 const double INF = 1e18;
 std::vector<double> g(W * H, INF);
 std::vector<int> parent(W * H, -1);
 std::vector<char> closed(W * H, 0);
 auto heur = [&](int x, int y) {
  double dx = std::abs(x - gx), dy = std::abs(y - gy);
  double oct = std::max(dx, dy) + (std::sqrt(2) - 1) * std::min(dx, dy);
  return oct * cm.res() * h_weight;
 };
 using Node = std::pair<double, int>; // f, idx
 std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
 g[idx(sx, sy)] = 0;
 open.emplace(heur(sx, sy), idx(sx, sy));
 const int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1}, DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};
 const double C[8] = {1, 1.4142, 1, 1.4142, 1, 1.4142, 1, 1.4142};
 while (!open.empty()) {
  auto [f, u] = open.top();
  open.pop();
  int ux = u % W, uy = u / W;
  if (closed[u])
   continue;
  closed[u] = 1;
  r.expanded++;
  if (ux == gx && uy == gy) {
   r.ok = true;
   for (int cur = u; cur != -1; cur = parent[cur]) {
    int cx = cur % W, cy = cur / W;
    r.path.push_back(cm.gridToWorld(cx, cy));
   }
   std::reverse(r.path.begin(), r.path.end());
   return r;
  }
  for (int k = 0; k < 8; k++) {
   int vx = ux + DX[k], vy = uy + DY[k];
   if (!cm.inside(vx, vy) || !cm.isFreeCell(vx, vy) || closed[idx(vx, vy)])
    continue;
   double ng = g[u] + C[k] * cm.res();
   if (ng < g[idx(vx, vy)]) {
    g[idx(vx, vy)] = ng;
    parent[idx(vx, vy)] = u;
    open.emplace(ng + heur(vx, vy), idx(vx, vy));
   }
  }
 }
 return r;
}
} // namespace sim
