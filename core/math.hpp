#pragma once
#include <cmath>
namespace sim {
struct Vec2 { double x{0}, y{0}; };
inline double clamp(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
inline double wrap_angle(double a){ while(a> M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }
}
