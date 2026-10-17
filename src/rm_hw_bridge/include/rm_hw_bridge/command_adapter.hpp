#ifndef RM_HW_BRIDGE__COMMAND_ADAPTER_HPP_
#define RM_HW_BRIDGE__COMMAND_ADAPTER_HPP_
#include "rm_hw_bridge/protocol.hpp"
namespace rm_hw_bridge {
constexpr double PI=3.14159265358979323846;
struct GimbalInput { double yaw=0,pitch=0; bool active=false,fire=false; };
struct NavInput { double x=0,y=0; };
struct AdapterConfig {
  bool allow_control=false,allow_shoot=false,allow_friction=false,allow_nav=false;
  double yaw_error_sign=1,pitch_error_sign=1,correction_scale=1,max_error_deg=5;
  uint64_t command_timeout_ms=100,nav_timeout_ms=100,feedback_timeout_ms=100;
  uint16_t valid_for_ms=100;
};
class CommandAdapter {
 public:
  explicit CommandAdapter(AdapterConfig config={}) : config_(config) {}
  void disconnect() { have_boot_=have_link_=have_feedback_=false; clear_commands(); }
  void clear_commands() { have_gimbal_=have_nav_=false; }
  void set_gimbal(GimbalInput c,uint64_t now) {
    if (!std::isfinite(c.yaw)||!std::isfinite(c.pitch)) c={};
    gimbal_=c; gimbal_ms_=now; have_gimbal_=true;
  }
  void set_nav(NavInput c,uint64_t now) {
    if (!std::isfinite(c.x)||!std::isfinite(c.y)||!std::isfinite(float(c.x))||!std::isfinite(float(c.y))) c={};
    nav_=c; nav_ms_=now; have_nav_=true;
  }
  bool accept(const Frame& f,uint64_t now) {
    ImuSample s; LinkStatus link;
    if (!decode_imu(f,s) && !decode_link(f,link)) return false;
    if (!have_boot_ || f.boot_id!=boot_) {
      disconnect(); boot_=f.boot_id; have_boot_=true;
      seen_gimbal_=seen_chassis_=seen_link_=false;
    }
    bool* seen=f.type==GIMBAL_IMU?&seen_gimbal_:f.type==CHASSIS_IMU?&seen_chassis_:&seen_link_;
    uint16_t* seq=f.type==GIMBAL_IMU?&rx_gimbal_:f.type==CHASSIS_IMU?&rx_chassis_:&rx_link_;
    if (*seen && !newer(f.sequence,*seq)) return false;
    *seen=true; *seq=f.sequence;
    if (f.type==GIMBAL_IMU) {
      if (have_feedback_ && (s.epoch!=feedback_.epoch ||
          ((s.flags&SAMPLE_TIME_VALID) && (feedback_.flags&SAMPLE_TIME_VALID) &&
           uint32_t(s.sample_time_us-feedback_.sample_time_us)>=0x80000000u))) clear_commands();
      if (have_feedback_ && (s.flags&SAMPLE_TIME_VALID) &&
          (feedback_.flags&SAMPLE_TIME_VALID) && s.epoch==feedback_.epoch &&
          s.sample_time_us==feedback_.sample_time_us) {
        // An invalid repeat must still revoke a previously usable measurement.
        if (!s.control_valid()) { feedback_=s; clear_commands(); }
        return false;
      }
      feedback_=s; feedback_ms_=now; have_feedback_=true;
      if (!s.control_valid()) have_gimbal_=false;
    } else if (f.type==LINK_STATUS) {
      if (!have_link_) { aim_seq_=uint16_t(link.last_aim_seq+1); clear_commands(); }
      else if (link.last_reject==8 && !newer(aim_seq_,link.last_aim_seq)) {
        aim_seq_=uint16_t(link.last_aim_seq+1); clear_commands();
      }
      link_=link; link_ms_=now; have_link_=true;
    }
    return true;
  }
  bool feedback_fresh(uint64_t now) const { return fresh(have_feedback_,feedback_ms_,now,config_.feedback_timeout_ms); }
  bool ready(uint64_t now) const { return have_boot_ && fresh(have_link_,link_ms_,now,config_.feedback_timeout_ms); }
  const ImuSample& feedback() const { return feedback_; }
  const LinkStatus& link() const { return link_; }
  uint64_t feedback_age(uint64_t now) const { return have_feedback_ && now>=feedback_ms_?now-feedback_ms_:UINT64_MAX; }
  Frame aim_command(uint64_t now,bool stop=false) {
    Frame f{AIM_COMMAND,aim_seq_++,boot_,std::vector<uint8_t>(24,0)};
    auto* p=f.payload.data();
    write_u16(p+4,feedback_.epoch); write_u16(p+6,feedback_.sequence);
    write_u32(p+8,feedback_.sample_time_us); write_u16(p+12,config_.valid_for_ms);
    if (!stop && ready(now) && config_.allow_control && feedback_fresh(now) && feedback_.control_valid() &&
        gimbal_.active && fresh(have_gimbal_,gimbal_ms_,now,config_.command_timeout_ms) &&
        feedback_age(now)<config_.valid_for_ms) {
      p[14]=3;
      if (config_.allow_friction) p[14]|=4;
      if (config_.allow_friction && config_.allow_shoot && gimbal_.fire) p[14]|=8;
      write_float(p+16,correction(std::remainder(gimbal_.yaw-feedback_.yaw,2*PI),config_.yaw_error_sign));
      write_float(p+20,correction(gimbal_.pitch-feedback_.pitch,config_.pitch_error_sign));
    }
    return f;
  }
  // Firmware has no NAV sequence acknowledgement. Stream restart can require an MCU reset.
  Frame nav_command(uint64_t now,bool stop=false) {
    Frame f{NAV_COMMAND,nav_seq_++,boot_,std::vector<uint8_t>(24,0)};
    auto* p=f.payload.data(); write_u32(p+4,feedback_.sample_time_us);
    write_u16(p+8,config_.valid_for_ms);
    if (!stop && config_.allow_nav && ready(now) &&
        fresh(have_nav_,nav_ms_,now,config_.nav_timeout_ms)) {
      p[10]=1; write_float(p+12,float(nav_.x)); write_float(p+16,float(nav_.y));
    }
    return f; // wz=0: RC::ApplyNav currently ignores this field.
  }
  bool nav_enabled() const { return config_.allow_nav; }
 private:
  static bool fresh(bool seen,uint64_t stamp,uint64_t now,uint64_t timeout) {
    return seen && now>=stamp && now-stamp<timeout;
  }
  float correction(double error,double sign) const {
    const double limit=config_.max_error_deg*PI/180;
    return float(std::clamp(error*sign*config_.correction_scale,-limit,limit));
  }
  AdapterConfig config_;
  GimbalInput gimbal_; NavInput nav_; ImuSample feedback_; LinkStatus link_;
  uint32_t boot_=0;
  uint16_t aim_seq_=0,nav_seq_=0,rx_gimbal_=0,rx_chassis_=0,rx_link_=0;
  uint64_t gimbal_ms_=0,nav_ms_=0,feedback_ms_=0,link_ms_=0;
  bool have_boot_=false,have_link_=false,have_feedback_=false,have_gimbal_=false,have_nav_=false;
  bool seen_gimbal_=false,seen_chassis_=false,seen_link_=false;
};
} // namespace rm_hw_bridge
#endif
