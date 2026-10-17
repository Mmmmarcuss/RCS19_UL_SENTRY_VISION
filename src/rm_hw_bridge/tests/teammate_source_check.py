"""Build unchanged MCU xuc/CRC sources with host UART/clock/IMU doubles.

Exercises actual firmware Encode/Decode and upper V2 code; no serial hardware.
Run in Linux: python3 teammate_source_check.py [path/to/Hohokam_SentinelRobot1009]
"""
from pathlib import Path
import os
import shutil
import subprocess
import sys


def main():
    project = Path(__file__).resolve().parents[3]
    lower = Path(sys.argv[1]) if len(sys.argv) > 1 else project.parent / 'Hohokam_SentinelRobot1009'
    build = Path(os.environ.get('RM_HW_BRIDGE_TEST_ROOT', project / '.cache')) / 'v2_firmware_host'
    build.mkdir(parents=True, exist_ok=True)
    for name in ('xuc.cpp', 'xuc.h', 'CRC.cpp', 'CRC.h'):
        shutil.copyfile(lower / 'STM32F405' / name, build / name)
    (build / 'host.hpp').write_text(r'''
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
using QueueHandle_t=void*;
constexpr unsigned UART_MAX_LEN=100;
constexpr int pdTRUE=1;
struct USART_TypeDef {};
inline uint32_t ticks=1000, micros=1000000;
inline uint32_t HAL_GetTick() { return ticks; }
inline uint32_t xTaskGetTickCount() { return ticks; }
struct TIM { static uint32_t NowUs() { return micros; } };
inline std::array<uint8_t,100> incoming{};
inline uint16_t incoming_size=0;
inline uint32_t incoming_time=0;
inline bool pending=false;
inline std::vector<uint8_t> transmitted;
inline int xQueueReceive(QueueHandle_t,void* dest,int) {
  if (!pending) return 0;
  std::memcpy(dest,incoming.data(),100); pending=false; return 1;
}
struct UART {
  QueueHandle_t UartQueueHandler=reinterpret_cast<void*>(1);
  UART& Init(USART_TypeDef*,uint32_t) { return *this; }
  UART& DMARxInit(void*) { return *this; }
  UART& DMATxInit() { return *this; }
  uint32_t GetRxTimeUs() { return incoming_time; }
  uint16_t GetRxLen() { return incoming_size; }
  void DMATransmit(uint8_t* p,uint32_t n) { transmitted.assign(p,p+n); }
};
struct IMU {
  float r=5,p=-10,y=179,gx=1,gy=2,gz=3;
  bool valid=true;
  bool IsValid() { return valid; }
  float GetAngleRoll() { return r; }
  float GetAnglePitch() { return p; }
  float GetAngleYaw() { return y; }
  float GetGyroRoll() { return gx; }
  float GetGyroPitch() { return gy; }
  float GetGyroYaw() { return gz; }
};
inline IMU imu_pantile,imu_chassis;
struct CONTROL { enum { AUTO,MANUAL }; int mode=AUTO; };
inline CONTROL ctrl;
''')
    for name in ('stm32f4xx_hal.h', 'usart.h', 'FreeRTOS.h', 'label.h', 'imu.h', 'control.h', 'tim.h'):
        (build / name).write_text('#pragma once\n#include "host.hpp"\n')
    (build / 'check.cpp').write_text(r'''
#include <cstdlib>
#include <iostream>
#include "xuc.h"
#include "rm_hw_bridge/command_adapter.hpp"
namespace hw=rm_hw_bridge;
#define CHECK(x) do { if (!(x)) { std::cerr<<"FAIL line "<<__LINE__<<": "<<#x<<'\n'; std::exit(1); } } while(0)
void send(const std::vector<uint8_t>& b) {
  CHECK(b.size()<=100);
  incoming.fill(0); std::copy(b.begin(),b.end(),incoming.begin());
  incoming_size=uint16_t(b.size()); incoming_time=micros; pending=true; xuc.Decode();
}
int main() {
  UART uart; USART_TypeDef instance; xuc.Init(&uart,&instance,921600);
  xuc.Encode();
  CHECK(transmitted.size()==106); // 46 + 34 + 26 in a single MCU write.
  hw::StateParser parser; std::vector<hw::Frame> frames;
  parser.feed(transmitted.data(),transmitted.size(),[&](const auto& f){frames.push_back(f);});
  CHECK(frames.size()==3 && frames[0].type==1 && frames[1].type==2 && frames[2].type==0x81);
  hw::ImuSample g,c;
  CHECK(hw::decode_imu(frames[0],g) && hw::decode_imu(frames[1],c));
  CHECK(g.sensor_id==1 && c.sensor_id==2 && g.flags==3 && c.flags==1);
  CHECK(g.sample_time_us==0 && c.sample_time_us==0); // Existing firmware timestamp limitation.
  CHECK(std::abs(g.yaw-179*hw::PI/180)<1e-6 && std::abs(g.gy-2*hw::PI/180)<1e-6);
  hw::AdapterConfig cfg; cfg.allow_control=true; cfg.allow_nav=true; cfg.max_error_deg=30;
  hw::CommandAdapter upper(cfg);
  for (const auto& f:frames) CHECK(upper.accept(f,1000));
  upper.set_gimbal({-179*hw::PI/180,0.1,true,true},1001);
  auto stop=upper.aim_command(1002);
  CHECK(stop.payload[14]==0); // Honest firmware flags cannot enable control.
  send(hw::encode(stop)); CHECK(xuc.last_reject==XUC_OK && !xuc.aim_control_en);
  send(hw::encode(stop)); CHECK(xuc.last_reject==XUC_DUPLICATE_OR_OLD);
  // Prime firmware's own cached reference using a harmless downlink RX timestamp.
  ++micros; ++ticks; send(hw::encode(upper.aim_command(1003,true))); xuc.Encode();
  frames.clear(); parser.feed(transmitted.data(),transmitted.size(),[&](const auto& f){frames.push_back(f);});
  // Only this test fixture marks validated time/axes to exercise later control encoding.
  frames[0].payload[1]=15;
  for (const auto& f:frames) CHECK(upper.accept(f,1010));
  upper.set_gimbal({-179*hw::PI/180,0.1,true,true},1011);
  auto aim=upper.aim_command(1012);
  CHECK(aim.payload[14]==3); // No friction or fire unless explicitly permitted.
  send(hw::encode(aim));
  CHECK(xuc.aim_accepted && xuc.last_reject==XUC_NOT_ALLOWED && !xuc.AimReady());
  CHECK(std::abs(xuc.aim_yaw_target+179*hw::PI/180)<1e-5);
  CHECK(std::abs(xuc.aim_pitch_target-0.1)<1e-5);
  auto wrong=aim; ++wrong.sequence; wrong.boot_id^=1;
  send(hw::encode(wrong)); CHECK(xuc.last_reject==XUC_BAD_SESSION);
  wrong=aim; ++wrong.sequence; hw::write_u16(wrong.payload.data()+6,32000);
  send(hw::encode(wrong)); CHECK(xuc.last_reject==XUC_BAD_REFERENCE);
  auto corrupt=hw::encode(aim); corrupt[30]^=1; const auto errors=xuc.crc_err;
  send(corrupt); CHECK(xuc.crc_err==uint16_t(errors+1));
  ++micros; auto bytes=hw::encode(upper.aim_command(1013,true));
  send(std::vector<uint8_t>(bytes.begin(),bytes.begin()+7));
  send(std::vector<uint8_t>(bytes.begin()+7,bytes.end()));
  CHECK(!xuc.aim_control_en && xuc.last_reject==XUC_OK);
  upper.set_nav({0.4,-0.2},1013); auto nav=upper.nav_command(1014);
  send(hw::encode(nav)); CHECK(xuc.nav_accepted && xuc.NavReady());
  CHECK(xuc.nav_vx==float(0.4) && xuc.nav_vy==float(-0.2) && xuc.nav_wz==0);
  ticks+=101; CHECK(!xuc.NavReady());
  send(hw::encode(upper.nav_command(1015,true))); CHECK(!xuc.NavReady() && !xuc.nav_enable);
  // Both upper commands fit one lower UART snapshot.
  bytes=hw::encode(upper.aim_command(1016,true));
  const auto navstop=hw::encode(upper.nav_command(1016,true)); bytes.insert(bytes.end(),navstop.begin(),navstop.end());
  send(bytes); CHECK(bytes.size()==76 && xuc.last_reject==XUC_OK);
  std::cout<<"PASS: unchanged MCU Encode/Decode + CRC, IMU units/flags, exact references, STOP/replay/reject, NAV lease, 76-byte batch\n";
}
''')
    binary = build / 'firmware_check'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-I', str(build),
                    '-I', str(project / 'src/rm_hw_bridge/include'), str(build / 'check.cpp'),
                    str(build / 'xuc.cpp'), str(build / 'CRC.cpp'), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    main()
