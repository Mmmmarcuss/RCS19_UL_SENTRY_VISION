#ifndef RM_HW_BRIDGE__COMMAND_ADAPTER_HPP_
#define RM_HW_BRIDGE__COMMAND_ADAPTER_HPP_
#include <algorithm>
#include "rm_hw_bridge/protocol.hpp"
namespace rm_hw_bridge {
constexpr double PI = 3.14159265358979323846;
struct GimbalInput { double yaw = 0, pitch = 0; bool active = false, fire = false; };
struct NavInput { double x = 0, y = 0; };
struct AdapterConfig {
  bool allow_control = true, allow_shoot = false;
  double yaw_error_sign = 1, pitch_error_sign = 1;
  double correction_scale = 1, max_error_deg = 5;
  uint64_t command_timeout_ms = 150, nav_timeout_ms = 250, feedback_timeout_ms = 300;
};
class CommandAdapter {
 public:
  explicit CommandAdapter(AdapterConfig config = {}) : config_(config) {}
  void set_gimbal(GimbalInput c, uint64_t now) {
    if (!std::isfinite(c.yaw) || !std::isfinite(c.pitch)) c = {};
    gimbal_ = c; gimbal_ms_ = now; have_gimbal_ = true;
  }
  void set_nav(NavInput c, uint64_t now) {
    if (!std::isfinite(c.x) || !std::isfinite(c.y)) c = {};
    nav_ = c; nav_ms_ = now; have_nav_ = true;
  }
  void set_feedback(const NucToVisionFrame& s, uint64_t now) {
    feedback_ = s; feedback_ms_ = now; have_feedback_ = true;
  }
  bool feedback_fresh(uint64_t now) const {
    return fresh(have_feedback_, feedback_ms_, now, config_.feedback_timeout_ms);
  }
  VisionToNucFrame command(uint64_t now) const {
    VisionToNucFrame c;
    if (feedback_fresh(now)) {
      c.pitch = feedback_.pitch;
      c.yaw = feedback_.yaw;
    }
    if (!config_.allow_control) return c;
    if (fresh(have_nav_, nav_ms_, now, config_.nav_timeout_ms)) {
      c.speed_x = float(nav_.x); c.speed_y = float(nav_.y);
      if (!std::isfinite(c.speed_x) || !std::isfinite(c.speed_y)) c.speed_x = c.speed_y = 0;
    }
    if (feedback_fresh(now) && gimbal_.active &&
        fresh(have_gimbal_, gimbal_ms_, now, config_.command_timeout_ms)) {
      c.yaw_diff = correction(std::remainder(gimbal_.yaw-feedback_.yaw*PI/180, 2*PI), config_.yaw_error_sign);
      c.pitch_diff = correction(gimbal_.pitch-feedback_.pitch*PI/180, config_.pitch_error_sign);
      c.fireadvice = config_.allow_shoot && gimbal_.fire ? 1 : 0;
    }
    return c;
  }
 private:
  static bool fresh(bool seen, uint64_t stamp, uint64_t now, uint64_t timeout) {
    return seen && now >= stamp && now-stamp < timeout;
  }
  float correction(double error, double sign) const {
    return float(std::clamp(error*180/PI*sign*config_.correction_scale,
      -config_.max_error_deg, config_.max_error_deg));
  }
  AdapterConfig config_;
  GimbalInput gimbal_;
  NavInput nav_;
  NucToVisionFrame feedback_;
  uint64_t gimbal_ms_ = 0, nav_ms_ = 0, feedback_ms_ = 0;
  bool have_gimbal_ = false, have_nav_ = false, have_feedback_ = false;
};
inline uint16_t counter_delta(uint16_t current, uint16_t previous) {
  return uint16_t(current-previous);
}
} // namespace rm_hw_bridge
#endif
