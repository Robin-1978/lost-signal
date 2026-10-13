#pragma once
#include <cmath>
namespace sim {
inline constexpr double kPi = 3.14159265358979323846;
struct Vec2 { double x{0}, y{0}; };
inline double clamp(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
inline double wrap_angle(double a){ while(a>kPi)a-=2*kPi; while(a<-kPi)a+=2*kPi; return a; }
}
