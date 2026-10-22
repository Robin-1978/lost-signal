#pragma once

// World <-> screen mapping. Pure math, no raylib dependency, so unit tests
// drive it without a window. Screen origin is top-left, +x right, +y down;
// world is +x right, +y up (y flips on projection).

namespace lost_signal::sim::viewer {

struct ScreenPoint {
  float x_px = 0.0f;
  float y_px = 0.0f;
};

class ViewTransform {
 public:
  ViewTransform() = default;
  ViewTransform(float scale_px_per_m, double center_x_m, double center_y_m,
                int screen_w_px, int screen_h_px)
      : scale_(scale_px_per_m),
        cx_m_(center_x_m),
        cy_m_(center_y_m),
        w_px_(screen_w_px),
        h_px_(screen_h_px) {}

  float scale() const { return scale_; }
  void set_screen(int w_px, int h_px) {
    w_px_ = w_px;
    h_px_ = h_px;
  }

  ScreenPoint world_to_screen(double x_m, double y_m) const {
    return ScreenPoint{
        static_cast<float>((x_m - cx_m_) * scale_ + w_px_ * 0.5),
        static_cast<float>((cy_m_ - y_m) * scale_ + h_px_ * 0.5),
    };
  }
  // Returns false when the transform is degenerate (no size or no scale).
  bool screen_to_world(float x_px, float y_px, double& x_m,
                       double& y_m) const {
    if (!(scale_ > 0.0f) || w_px_ <= 0 || h_px_ <= 0) return false;
    x_m = (x_px - w_px_ * 0.5) / scale_ + cx_m_;
    y_m = cy_m_ - (y_px - h_px_ * 0.5) / scale_;
    return true;
  }

  void pan_px(float dx_px, float dy_px) {
    if (!(scale_ > 0.0f)) return;
    cx_m_ -= dx_px / scale_;
    cy_m_ += dy_px / scale_;
  }
  // Zoom keeping the world point under the cursor fixed.
  void zoom_at(float x_px, float y_px, float factor) {
    if (!(factor > 0.0f) || !(scale_ > 0.0f)) return;
    double wx = 0.0, wy = 0.0;
    if (!screen_to_world(x_px, y_px, wx, wy)) return;
    scale_ *= factor;
    if (scale_ < 1.0f) scale_ = 1.0f;
    if (scale_ > 2000.0f) scale_ = 2000.0f;
    cx_m_ = wx - (x_px - w_px_ * 0.5) / scale_;
    cy_m_ = wy + (y_px - h_px_ * 0.5) / scale_;
  }
  // Frame all of (min/max world) with a margin; false when empty/degen.
  bool fit(double x0_m, double y0_m, double x1_m, double y1_m,
           float margin_px = 40.0f) {
    if (w_px_ <= 0 || h_px_ <= 0) return false;
    const double dx = x1_m - x0_m, dy = y1_m - y0_m;
    if (!(dx >= 0.0) || !(dy >= 0.0)) return false;
    const float sx =
        static_cast<float>((w_px_ - 2 * margin_px) / (dx > 1e-6 ? dx : 1e-6));
    const float sy =
        static_cast<float>((h_px_ - 2 * margin_px) / (dy > 1e-6 ? dy : 1e-6));
    scale_ = sx < sy ? sx : sy;
    if (!(scale_ > 0.0f)) return false;
    cx_m_ = (x0_m + x1_m) * 0.5;
    cy_m_ = (y0_m + y1_m) * 0.5;
    return true;
  }

 private:
  float scale_ = 50.0f;  // px per meter
  double cx_m_ = 0.0, cy_m_ = 0.0;
  int w_px_ = 0, h_px_ = 0;
};

}  // namespace lost_signal::sim::viewer
