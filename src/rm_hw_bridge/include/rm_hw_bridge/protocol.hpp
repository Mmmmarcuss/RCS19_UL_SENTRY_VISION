#ifndef RM_HW_BRIDGE__PROTOCOL_HPP_
#define RM_HW_BRIDGE__PROTOCOL_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#include "rm_hw_bridge/crc16.hpp"

namespace rm_hw_bridge {
// Hohokam SentinelRobot 8caff9c: V2, little endian, radians, CRC16/MCRF4XX.
constexpr uint8_t GIMBAL_IMU=0x01, CHASSIS_IMU=0x02, AIM_COMMAND=0x10,
                  NAV_COMMAND=0x11, LINK_STATUS=0x81;
constexpr uint8_t ORIENTATION_VALID=1, GYRO_VALID=2, SAMPLE_TIME_VALID=4, AXES_VERIFIED=8;
constexpr size_t HEADER_SIZE=12, MAX_PAYLOAD=128, MAX_FRAME=142;
static_assert(sizeof(float)==4 && std::numeric_limits<float>::is_iec559);
inline uint16_t read_u16(const uint8_t* p) { return uint16_t(p[0]) | uint16_t(p[1])<<8; }
inline uint32_t read_u32(const uint8_t* p) {
  return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24;
}
inline void write_u16(uint8_t* p, uint16_t v) { p[0]=uint8_t(v); p[1]=uint8_t(v>>8); }
inline void write_u32(uint8_t* p, uint32_t v) {
  for (unsigned i=0;i<4;++i) p[i]=uint8_t(v>>(8*i));
}
inline float read_float(const uint8_t* p) {
  uint32_t bits=read_u32(p); float v; std::memcpy(&v,&bits,4); return v;
}
inline void write_float(uint8_t* p, float v) {
  uint32_t bits; std::memcpy(&bits,&v,4); write_u32(p,bits);
}
inline bool newer(uint16_t a, uint16_t b) { const auto d=uint16_t(a-b); return d!=0 && d<32768; }
struct Frame {
  uint8_t type=0;
  uint16_t sequence=0;
  uint32_t boot_id=0;
  std::vector<uint8_t> payload;
};
inline std::vector<uint8_t> encode(const Frame& f) {
  if (f.payload.size()>MAX_PAYLOAD) return {};
  std::vector<uint8_t> b(14+f.payload.size(),0);
  b[0]=0xA6; b[1]=0x6A; b[2]=2; b[3]=f.type;
  write_u16(b.data()+4,uint16_t(f.payload.size())); write_u16(b.data()+6,f.sequence);
  write_u32(b.data()+8,f.boot_id);
  std::copy(f.payload.begin(),f.payload.end(),b.begin()+12);
  write_u16(b.data()+b.size()-2,compute_crc16(b.data(),b.size()-2)); return b;
}
struct ImuSample {
  uint8_t sensor_id=0, flags=0;
  uint16_t epoch=0, sequence=0;
  uint32_t sample_time_us=0, boot_id=0;
  float roll=0,pitch=0,yaw=0,gx=0,gy=0,gz=0;
  bool orientation_valid() const {
    return (flags&ORIENTATION_VALID) && std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
  }
  bool gyro_valid() const {
    return sensor_id==1 && (flags&GYRO_VALID) && std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gz);
  }
  bool control_valid() const {
    return orientation_valid() && (flags&SAMPLE_TIME_VALID) && (flags&AXES_VERIFIED);
  }
};
inline bool decode_imu(const Frame& f, ImuSample& s) {
  const bool g=f.type==GIMBAL_IMU;
  if ((!g && f.type!=CHASSIS_IMU) || f.payload.size()!=(g?32u:20u)) return false;
  const auto* p=f.payload.data();
  if (p[0]!=(g?1:2) || (p[1]&0xF0) || (!g && (p[1]&GYRO_VALID))) return false;
  s={}; s.sensor_id=p[0]; s.flags=p[1]; s.epoch=read_u16(p+2);
  s.sample_time_us=read_u32(p+4); s.sequence=f.sequence; s.boot_id=f.boot_id;
  s.roll=read_float(p+8); s.pitch=read_float(p+12); s.yaw=read_float(p+16);
  if (g) { s.gx=read_float(p+20); s.gy=read_float(p+24); s.gz=read_float(p+28); }
  // Invalid measurements reach the health layer to revoke previous validity.
  return true;
}
struct LinkStatus {
  uint16_t last_aim_seq=0,aim_age_ms=0xFFFF;
  uint8_t state=0,last_reject=0;
};
inline bool decode_link(const Frame& f, LinkStatus& s) {
  if (f.type!=LINK_STATUS || f.payload.size()!=12) return false;
  const auto* p=f.payload.data(); s.last_aim_seq=read_u16(p+4);
  s.state=p[6]; s.last_reject=p[7]; s.aim_age_ms=read_u16(p+8); return true;
}
// Bounded stream parser; a stalled partial frame expires on monotonic time.
class StateParser {
 public:
  template<class Consume> void feed(const uint8_t* data,size_t n,Consume consume,uint64_t now_ms=0) {
    if (!data && n) return;
    if (!buffer_.empty() && now_ms>last_ms_ && now_ms-last_ms_>100) buffer_.clear();
    last_ms_=now_ms;
    for (size_t i=0;i<n;++i) {
      buffer_.push_back(data[i]);
      while (buffer_.size()>=2) {
        if (buffer_[0]!=0xA6 || buffer_[1]!=0x6A) { discard(); continue; }
        if (buffer_.size()<HEADER_SIZE) break;
        const size_t plen=read_u16(buffer_.data()+4);
        if (buffer_[2]!=2 || plen>MAX_PAYLOAD) { ++rejected_; discard(); continue; }
        const size_t total=14+plen;
        if (buffer_.size()<total) break;
        if (read_u16(buffer_.data()+total-2)!=compute_crc16(buffer_.data(),total-2)) {
          ++rejected_; discard(); continue;
        }
        Frame f{buffer_[3],read_u16(buffer_.data()+6),read_u32(buffer_.data()+8),
                std::vector<uint8_t>(buffer_.begin()+12,buffer_.begin()+total-2)};
        buffer_.erase(buffer_.begin(),buffer_.begin()+total); ++valid_; consume(f);
      }
    }
  }
  uint64_t valid_frames() const { return valid_; }
  uint64_t rejected_frames() const { return rejected_; }
 private:
  void discard() { buffer_.erase(buffer_.begin()); }
  std::vector<uint8_t> buffer_;
  uint64_t last_ms_=0,valid_=0,rejected_=0;
};
} // namespace rm_hw_bridge
#endif
