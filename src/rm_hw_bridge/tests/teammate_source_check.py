"""Compile unchanged firmware Encode/Decode bodies against the upper protocol.

UART/FreeRTOS/IMU are host doubles; this checks wire compatibility, not MCU timing.
Usage: python3 teammate_source_check.py [path/to/Hohokam_SentinelRobot1009]
"""
from pathlib import Path
import re
import subprocess
import sys


def function(source, name):
    start = source.index(f"void XUC::{name}()")
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    project = Path(__file__).resolve().parents[3]
    lower = Path(sys.argv[1]) if len(sys.argv) > 1 else project.parent / "Hohokam_SentinelRobot1009"
    lower = lower.resolve()
    header = (lower / "STM32F405/xuc.h").read_text(encoding="utf-8-sig")
    source = (lower / "STM32F405/xuc.cpp").read_text(encoding="utf-8-sig")
    crc_source = (lower / "STM32F405/CRC.cpp").read_text(encoding="utf-8-sig")
    tx_packet = re.search(r"struct TxPacket\s*\{.*?\}__attribute__\(\(packed\)\);", header, re.S).group()
    frame_len = re.search(r"#define XUC_FRAME_LEN\s+\d+", header).group()
    crc_source = re.sub(r'^#include "CRC.h"\s*', '', crc_source)
    build = project / ".cache/teammate_source_check"
    build.mkdir(parents=True, exist_ok=True)
    cpp = r'''
#include <array>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "rm_hw_bridge/protocol.hpp"
#define CHECK(x) do { if (!(x)) { std::cerr << "FAIL line " << __LINE__ << ": " << #x << '\n'; std::exit(1); } } while (0)
namespace firmware {
''' + frame_len + '\n' + crc_source + '\n' + tx_packet + r'''
constexpr float PI = 3.14159265358979323846f;
constexpr int pdTRUE = 1;
std::array<uint8_t, 100> incoming{};
bool pending = false;
std::array<uint8_t, 32> transmitted{};
int xQueueReceive(int, uint8_t* destination, int) {
  if (!pending) return 0;
  std::memcpy(destination, incoming.data(), incoming.size()); pending = false; return 1;
}
struct UART {
  int UartQueueHandler = 1;
  void DMATransmit(uint8_t* data, int size) {
    CHECK(size == 32); std::memcpy(transmitted.data(), data, size);
  }
} uart;
struct { struct { struct { uint8_t robot_id = 7; } robot_status_t; } data; } judgement;
struct IMU {
  float GetAngleRoll() { return 20; }
  float GetAnglePitch() { return -10; }
  float GetAngleYaw() { return 179; }
} imu_pantile;
class XUC {
 public:
  UART* m_uart = &uart;
  uint8_t m_frame[100]{}, tx_data[64]{};
  int pd_Rx = 0;
  uint16_t rx_total = 0, rx_count = 0, crc_err = 0;
  float yaw = 0, yaw_pre = 0, yaw_spd = 0, pitch = 0;
  float yaw_diff = 0, pitch_diff = 0, distance = 0, v_y = 0, speed_x = 0, speed_y = 0;
  float aim_x = 1, aim_y = 2, aim_z = 3;
  bool fireadvice = false;
  enum ARMOR_COLOR { RED = 0, BLUE }; ARMOR_COLOR own_color = RED;
  TxPacket TxNuc;
  float FR4(uint8_t* p) { float f; std::memcpy(&f, p, 4); return f; }
  void Decode(); void Encode();
};
''' + function(source, 'Decode') + '\n' + function(source, 'Encode') + r'''
} // namespace firmware
int main() {
  namespace hw = rm_hw_bridge;
  firmware::XUC xuc;
  static_assert(sizeof(firmware::TxPacket) == 32);
  static_assert(offsetof(firmware::TxPacket, roll) == 2);
  static_assert(offsetof(firmware::TxPacket, rx_total) == 26);
  static_assert(offsetof(firmware::TxPacket, rx_count) == 28);
  static_assert(offsetof(firmware::TxPacket, checksum) == 30);
  constexpr double D2R = 3.14159265358979323846/180;
  for (int i = 0; i < 1000; ++i) {
    hw::VisionToNucFrame c;
    c.pitch = float(i%61-30); c.yaw = float(i%361-180);
    c.yaw_diff = float(i%11-5); c.pitch_diff = -c.yaw_diff;
    c.distance = i*0.01f; c.v_y = -0.05f*i;
    c.speed_x = 0.001f*i; c.speed_y = -0.002f*i; c.fireadvice = i&1;
    auto wire = hw::encode(c);
    std::memcpy(firmware::incoming.data(), wire.data(), wire.size()); firmware::pending = true;
    xuc.Decode();
    CHECK(xuc.rx_total == i+1 && xuc.rx_count == i+1 && xuc.crc_err == 0);
    CHECK(std::abs(xuc.pitch-c.pitch*D2R) < 1e-5 && std::abs(xuc.yaw-c.yaw*D2R) < 1e-5);
    CHECK(std::abs(xuc.yaw_diff-c.yaw_diff*D2R) < 1e-5);
    CHECK(std::abs(xuc.pitch_diff-c.pitch_diff*D2R) < 1e-5);
    CHECK(xuc.distance == c.distance && xuc.v_y == c.v_y);
    CHECK(xuc.speed_x == c.speed_x && xuc.speed_y == c.speed_y && xuc.fireadvice == bool(c.fireadvice));
  }
  firmware::incoming[9] ^= 1; firmware::pending = true; xuc.Decode();
  CHECK(xuc.rx_total == 1001 && xuc.rx_count == 1000 && xuc.crc_err == 1);
  firmware::incoming[0] = 0; firmware::pending = true; xuc.Decode();
  CHECK(xuc.rx_total == 1002 && xuc.rx_count == 1000 && xuc.crc_err == 1);
  for (auto id : {7, 107}) {
    firmware::judgement.data.robot_status_t.robot_id = id;
    xuc.Encode();
    hw::NucToVisionFrame s;
    CHECK(hw::decode(firmware::transmitted.data(), 32, s));
    CHECK(s.roll == 20 && s.pitch == -10 && s.yaw == 179);
    CHECK(s.aim_x == 1 && s.aim_y == 2 && s.aim_z == 3);
    CHECK(s.rx_total == 1002 && s.rx_count == 1000);
    CHECK(s.flags == (id == 7 ? 61 : 60));
  }
  std::cout << "PASS: 1000 upper frames accepted by actual firmware Decode; CRC/header rejection; actual firmware Encode parsed by upper; STATE32 offsets and flag bits\n";
}
'''
    cpp_path = build / 'source_check.cpp'
    cpp_path.write_text(cpp, encoding='utf-8')
    binary = build / 'source_check'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-I', str(project / 'src/rm_hw_bridge/include'),
                    str(cpp_path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
