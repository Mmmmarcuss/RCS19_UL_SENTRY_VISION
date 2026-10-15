#ifndef RM_HW_BRIDGE__PROTOCOL_HPP_
#define RM_HW_BRIDGE__PROTOCOL_HPP_
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include "rm_hw_bridge/crc16.hpp"

namespace rm_hw_bridge {
// Hohokamk/Sentinel_robot 6a3fc9ea454d: COMMAND40 / STATE32.
#pragma pack(push, 1)
struct VisionToNucFrame {
  uint8_t header = 0xA5;
  float pitch = 0, yaw = 0;             // Current pose, degrees.
  float yaw_diff = 0, pitch_diff = 0;   // Remaining error, degrees.
  float distance = 0;
  uint8_t fireadvice = 0, reserved[3]{};
  float v_y = 0;
  uint8_t reserved2 = 0;
  float speed_x = 0, speed_y = 0;       // Body-frame m/s.
  uint16_t checksum = 0;
};
struct NucToVisionFrame {
  uint8_t header = 0x5A, flags = 0;
  float roll = 0, pitch = 0, yaw = 0;   // CH010 HI91 raw degrees (firmware does not convert).
  float aim_x = 0, aim_y = 0, aim_z = 0;
  uint16_t rx_total = 0, rx_count = 0, checksum = 0;
};
#pragma pack(pop)
constexpr size_t VISION_TX_FRAME_SIZE = 40, VISION_RX_FRAME_SIZE = 32;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(sizeof(VisionToNucFrame) == VISION_TX_FRAME_SIZE);
static_assert(sizeof(NucToVisionFrame) == VISION_RX_FRAME_SIZE);
#define RM_OFFSET(T, F, N) static_assert(offsetof(T, F) == N)
RM_OFFSET(VisionToNucFrame, header, 0);
RM_OFFSET(VisionToNucFrame, pitch, 1); RM_OFFSET(VisionToNucFrame, yaw, 5);
RM_OFFSET(VisionToNucFrame, yaw_diff, 9); RM_OFFSET(VisionToNucFrame, pitch_diff, 13);
RM_OFFSET(VisionToNucFrame, distance, 17); RM_OFFSET(VisionToNucFrame, fireadvice, 21);
RM_OFFSET(VisionToNucFrame, reserved, 22); RM_OFFSET(VisionToNucFrame, v_y, 25);
RM_OFFSET(VisionToNucFrame, reserved2, 29); RM_OFFSET(VisionToNucFrame, speed_x, 30);
RM_OFFSET(VisionToNucFrame, speed_y, 34); RM_OFFSET(VisionToNucFrame, checksum, 38);
RM_OFFSET(NucToVisionFrame, flags, 1); RM_OFFSET(NucToVisionFrame, roll, 2);
RM_OFFSET(NucToVisionFrame, pitch, 6); RM_OFFSET(NucToVisionFrame, yaw, 10);
RM_OFFSET(NucToVisionFrame, aim_x, 14); RM_OFFSET(NucToVisionFrame, aim_y, 18);
RM_OFFSET(NucToVisionFrame, aim_z, 22); RM_OFFSET(NucToVisionFrame, rx_total, 26);
RM_OFFSET(NucToVisionFrame, rx_count, 28); RM_OFFSET(NucToVisionFrame, checksum, 30);
#undef RM_OFFSET
inline uint16_t read_u16(const uint8_t* p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}
inline void write_u16(uint8_t* p, uint16_t value) {
  p[0] = uint8_t(value); p[1] = uint8_t(value >> 8);
}
inline float read_float(const uint8_t* p) {
  const uint32_t bits = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
    (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
  float result; std::memcpy(&result, &bits, 4); return result;
}
inline void write_float(uint8_t* p, float value) {
  uint32_t bits; std::memcpy(&bits, &value, 4);
  for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(bits >> (8 * i));
}
inline std::array<uint8_t, VISION_TX_FRAME_SIZE> encode(const VisionToNucFrame& c) {
  std::array<uint8_t, VISION_TX_FRAME_SIZE> b{};
  b[0] = 0xA5;
  write_float(b.data()+1, c.pitch); write_float(b.data()+5, c.yaw);
  write_float(b.data()+9, c.yaw_diff); write_float(b.data()+13, c.pitch_diff);
  write_float(b.data()+17, c.distance); b[21] = c.fireadvice & 1;
  write_float(b.data()+25, c.v_y);
  write_float(b.data()+30, c.speed_x); write_float(b.data()+34, c.speed_y);
  write_u16(b.data()+38, compute_crc16(b.data(), 38));
  return b;
}
inline bool decode(const uint8_t* b, size_t n, NucToVisionFrame& s) {
  if (!b || n != VISION_RX_FRAME_SIZE || b[0] != 0x5A || !check_crc16(b, n)) return false;
  NucToVisionFrame value;
  value.flags = b[1]; value.roll = read_float(b+2);
  value.pitch = read_float(b+6); value.yaw = read_float(b+10);
  value.aim_x = read_float(b+14); value.aim_y = read_float(b+18);
  value.aim_z = read_float(b+22); value.rx_total = read_u16(b+26);
  value.rx_count = read_u16(b+28); value.checksum = read_u16(b+30);
  if (!std::isfinite(value.roll) || !std::isfinite(value.pitch) ||
      !std::isfinite(value.yaw)) return false;
  s = value; return true;
}
class StateParser {
 public:
  template<class Consumer> void feed(const uint8_t* data, size_t n, Consumer consume) {
    for (size_t i = 0; i < n; ++i) {
      if (!used_ && data[i] != 0x5A) continue;
      bytes_[used_++] = data[i];
      if (used_ < bytes_.size()) continue;
      NucToVisionFrame state;
      if (decode(bytes_.data(), bytes_.size(), state)) {
        ++valid_frames; consume(state); used_ = 0;
      } else {
        ++rejected_frames;
        size_t next = 1;
        while (next < used_ && bytes_[next] != 0x5A) ++next;
        used_ -= next;
        std::memmove(bytes_.data(), bytes_.data()+next, used_);
      }
    }
  }
  uint64_t valid_frames = 0, rejected_frames = 0;
 private:
  std::array<uint8_t, VISION_RX_FRAME_SIZE> bytes_{};
  size_t used_ = 0;
};
} // namespace rm_hw_bridge
#endif
