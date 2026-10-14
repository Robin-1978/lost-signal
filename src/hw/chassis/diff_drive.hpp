#pragma once
namespace sim {
// 差速底盘(MVP车型): 双轮差速, 可原地转向/倒车, 无最小转弯半径限制.
// w 过大易甩尾, 里程计滑移噪声会放大(见 motor_drive 牵引力钳制).
struct DiffState {
 double x{0}, y{0}, theta{0}, v{0}, w{0};
};
struct DiffCmd {
 double target_v{0}, target_w{0};
};
struct DiffParams {
 double v_max{2.0}, v_min{-1.0}, w_max{2.0};
 double accel_max{3.0}, alpha_max{8.0};
 double wheel_base{0.5};
};
class DiffDrive {
 public:
 explicit DiffDrive(DiffParams p = {}) : p_(p) {}
 DiffState step(const DiffState& s, const DiffCmd& cmd, double dt);
 const DiffParams& params() const { return p_; }
 // 给"换电机/轮胎"升级项预留: twist<->轮速
 static void twistToWheels(double v, double w, double base, double& vl, double& vr);
 static void wheelsToTwist(double vl, double vr, double base, double& v, double& w);

 private:
 DiffParams p_;
};
} // namespace sim
