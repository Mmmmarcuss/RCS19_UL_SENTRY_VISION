#include <cstdlib>
#include <iostream>
#include <vector>
#include "rm_hw_bridge/command_adapter.hpp"
namespace hw = rm_hw_bridge;
#define CHECK(x) do { if (!(x)) { std::cerr << "Failed line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while (0)
static uint16_t independent_crc(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; ++i) crc = (crc >> 1)^((crc & 1) ? 0x8408 : 0);
  }
  return crc;
}
static std::array<uint8_t, 32> state_bytes(float yaw = 0.5f) {
  std::array<uint8_t, 32> b{};
  b[0] = 0x5A; b[1] = (15 << 2) | 1;
  hw::write_float(b.data()+2, 0.2f); hw::write_float(b.data()+6, -0.1f);
  hw::write_float(b.data()+10, yaw); hw::write_float(b.data()+14, 1.5f);
  hw::write_float(b.data()+18, 2.5f); hw::write_float(b.data()+22, 3.5f);
  hw::write_u16(b.data()+26, 65535); hw::write_u16(b.data()+28, 123);
  hw::write_u16(b.data()+30, independent_crc(b.data(), 30)); return b;
}
int main() {
  const uint8_t known[] = "123456789";
  CHECK(hw::compute_crc16(known, 9) == 0x6F91);
  CHECK(!hw::check_crc16(nullptr, 0));
  hw::VisionToNucFrame c;
  c.pitch = 1.25; c.yaw = -2.5; c.yaw_diff = 3.75; c.pitch_diff = -4.5;
  c.distance = 6; c.v_y = -7; c.speed_x = 0.3; c.speed_y = -0.4; c.fireadvice = 1;
  const auto wire = hw::encode(c);
  CHECK(wire.size() == 40 && wire[0] == 0xA5 && wire[21] == 1);
  CHECK(wire[1] == 0 && wire[2] == 0 && wire[3] == 0xA0 && wire[4] == 0x3F);
  CHECK(hw::read_float(wire.data()+5) == -2.5f);
  CHECK(hw::read_float(wire.data()+9) == 3.75f);
  CHECK(hw::read_float(wire.data()+13) == -4.5f);
  CHECK(hw::read_float(wire.data()+17) == 6 && hw::read_float(wire.data()+25) == -7);
  CHECK(hw::read_float(wire.data()+30) == float(0.3));
  CHECK(hw::read_float(wire.data()+34) == float(-0.4));
  CHECK(wire[22] == 0 && wire[23] == 0 && wire[24] == 0 && wire[29] == 0);
  CHECK(hw::read_u16(wire.data()+38) == independent_crc(wire.data(), 38));
  const auto good = state_bytes();
  hw::NucToVisionFrame s;
  CHECK(hw::decode(good.data(), good.size(), s));
  CHECK(s.yaw == 0.5 && s.roll == 0.2f && s.pitch == -0.1f);
  CHECK(s.flags == 61 && s.aim_x == 1.5 && s.aim_y == 2.5 && s.aim_z == 3.5);
  CHECK(s.rx_total == 65535 && s.rx_count == 123);
  CHECK(!hw::decode(good.data(), 28, s));
  CHECK(!hw::decode(nullptr, 32, s));
  auto bad = good; bad[10] ^= 1;
  CHECK(!hw::decode(bad.data(), bad.size(), s));
  auto nan = state_bytes(std::numeric_limits<float>::quiet_NaN());
  CHECK(!hw::decode(nan.data(), nan.size(), s));
  for (size_t split = 0; split <= good.size(); ++split) {
    hw::StateParser parser; unsigned received = 0;
    auto consume = [&](const auto&) { ++received; };
    parser.feed(good.data(), split, consume);
    parser.feed(good.data()+split, good.size()-split, consume);
    CHECK(received == 1);
  }
  std::vector<uint8_t> stream{0, 1, 0x5A, 2, 3};
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), nan.begin(), nan.end());
  stream.insert(stream.end(), good.begin(), good.end());
  stream.insert(stream.end(), good.begin(), good.end());
  for (size_t chunk = 1; chunk <= stream.size(); ++chunk) {
    hw::StateParser parser; unsigned received = 0;
    for (size_t i = 0; i < stream.size(); i += chunk)
      parser.feed(stream.data()+i, std::min(chunk, stream.size()-i), [&](const auto&) { ++received; });
    CHECK(received == 2);
  }
  hw::AdapterConfig config; config.allow_shoot = true;
  config.max_error_deg = 30;
  hw::CommandAdapter adapter(config);
  hw::NucToVisionFrame pose; pose.yaw = 179; pose.pitch = float(-0.1*180/hw::PI);
  adapter.set_feedback(pose, 1000);
  adapter.set_nav({0.3, -0.4}, 1000);
  adapter.set_gimbal({-179*hw::PI/180, 0.1, true, true}, 1000);
  auto combined = adapter.command(1010);
  CHECK(std::abs(combined.yaw_diff-2) < 0.001);
  CHECK(std::abs(combined.pitch_diff-0.2*180/hw::PI) < 0.001);
  CHECK(std::abs(combined.yaw-179) < 0.001 && combined.fireadvice == 1);
  CHECK(combined.speed_x == float(0.3) && combined.speed_y == float(-0.4));
  combined = adapter.command(1150); // Gimbal expired, navigation remains fresh.
  CHECK(combined.fireadvice == 0 && combined.yaw_diff == 0 && combined.speed_x != 0);
  combined = adapter.command(1300); // All sources expired.
  CHECK(combined.fireadvice == 0 && combined.speed_x == 0 && combined.pitch_diff == 0);
  adapter.set_gimbal({-179*hw::PI/180, 0.1, true, true}, 1300);
  adapter.set_nav({0.3, -0.4}, 1300);
  combined = adapter.command(1300); // Fresh command cannot aim/fire from expired feedback.
  CHECK(combined.yaw_diff == 0 && combined.pitch_diff == 0 && combined.fireadvice == 0);
  CHECK(combined.speed_x == float(0.3));
  adapter.set_feedback(pose, 1300);
  adapter.set_gimbal({-179*hw::PI/180, 0.1, false, true}, 1300);
  CHECK(adapter.command(1300).yaw_diff == 0 && adapter.command(1300).fireadvice == 0);
  adapter.set_gimbal({std::numeric_limits<double>::quiet_NaN(), 0, true, true}, 1310);
  adapter.set_feedback(pose, 1310);
  CHECK(adapter.command(1310).fireadvice == 0);
  hw::CommandAdapter disabled;
  disabled.set_feedback(pose, 1); disabled.set_gimbal({1, 0, true, true}, 1);
  CHECK(disabled.command(1).fireadvice == 0);
  CHECK(std::abs(disabled.command(1).yaw_diff) <= 5);
  config.yaw_error_sign = config.pitch_error_sign = -1;
  hw::CommandAdapter inverted(config);
  inverted.set_feedback(pose, 1);
  inverted.set_gimbal({-179*hw::PI/180, 0.1, true, false}, 1);
  CHECK(std::abs(inverted.command(1).yaw_diff+2) < 0.001);
  CHECK(inverted.command(1).pitch_diff < 0);
  config.allow_control = false;
  hw::CommandAdapter no_control(config);
  no_control.set_feedback(pose, 1); no_control.set_nav({1, 1}, 1);
  no_control.set_gimbal({1, 0, true, true}, 1);
  CHECK(no_control.command(1).speed_x == 0 && no_control.command(1).fireadvice == 0);
  CHECK(hw::counter_delta(2, 65535) == 3);
  std::cout << "PASS: wire offsets/CRC, split/merged/noisy frames, invalid data, merged commands, wrap and timeouts\n";
}
