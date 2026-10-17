#include <cstdlib>
#include <iostream>
#include "rm_hw_bridge/command_adapter.hpp"
namespace hw=rm_hw_bridge;
#define CHECK(x) do { if (!(x)) { std::cerr<<"Failed line "<<__LINE__<<": "<<#x<<'\n'; std::exit(1); } } while (0)
static uint16_t independent_crc(const uint8_t* p,size_t n) {
  uint16_t c=0xFFFF;
  while (n--) { c^=*p++; for(int i=0;i<8;++i) c=(c>>1)^((c&1)?0x8408:0); }
  return c;
}
static hw::Frame sample(uint16_t seq=10,uint32_t time=1000000,uint8_t flags=15,uint8_t type=hw::GIMBAL_IMU) {
  hw::Frame f{type,seq,0x12345678,std::vector<uint8_t>(type==hw::GIMBAL_IMU?32:20,0)};
  auto* p=f.payload.data(); p[0]=type==hw::GIMBAL_IMU?1:2; p[1]=flags;
  hw::write_u16(p+2,1); hw::write_u32(p+4,time);
  hw::write_float(p+8,0.05f); hw::write_float(p+12,-0.1f); hw::write_float(p+16,float(179*hw::PI/180));
  if (type==hw::GIMBAL_IMU) {
    hw::write_float(p+20,1); hw::write_float(p+24,2); hw::write_float(p+28,3);
  }
  return f;
}
static hw::Frame link(uint16_t seq=1,uint16_t last=40000) {
  hw::Frame f{hw::LINK_STATUS,seq,0x12345678,std::vector<uint8_t>(12,0)};
  hw::write_u16(f.payload.data()+4,last); return f;
}
int main() {
  const uint8_t known[]="123456789";
  CHECK(hw::compute_crc16(known,9)==0x6F91);
  const auto good=hw::encode(sample());
  CHECK(good.size()==46 && good[0]==0xA6 && good[1]==0x6A && good[2]==2 && good[3]==1);
  CHECK(good[8]==0x78 && good[9]==0x56 && good[10]==0x34 && good[11]==0x12);
  CHECK(hw::read_u16(good.data()+44)==independent_crc(good.data(),44));
  auto bad=good; bad[30]^=1;
  std::vector<uint8_t> stream{0,1,0xA6,2,3};
  stream.insert(stream.end(),bad.begin(),bad.end());
  stream.insert(stream.end(),good.begin(),good.end());
  const auto base=hw::encode(sample(10,100,13,hw::CHASSIS_IMU));
  const auto status=hw::encode(link());
  CHECK(base.size()==34 && status.size()==26);
  stream.insert(stream.end(),base.begin(),base.end()); stream.insert(stream.end(),status.begin(),status.end());
  for(size_t chunk=1;chunk<=stream.size();++chunk) {
    hw::StateParser parser; unsigned received=0;
    for(size_t i=0;i<stream.size();i+=chunk)
      parser.feed(stream.data()+i,std::min(chunk,stream.size()-i),[&](const auto&){++received;});
    CHECK(received==3);
  }
  for(size_t split=0;split<=good.size();++split) {
    hw::StateParser p; unsigned n=0;
    p.feed(good.data(),split,[&](const auto&){++n;});
    p.feed(good.data()+split,good.size()-split,[&](const auto&){++n;}); CHECK(n==1);
  }
  hw::StateParser stalled; unsigned count=0;
  stalled.feed(good.data(),20,[&](const auto&){++count;},10);
  stalled.feed(good.data(),good.size(),[&](const auto&){++count;},111); CHECK(count==1);
  auto oversized=good; oversized[4]=129;
  stalled.feed(oversized.data(),12,[&](const auto&){++count;},112);
  stalled.feed(good.data(),good.size(),[&](const auto&){++count;},112); CHECK(count==2);
  hw::ImuSample s;
  CHECK(hw::decode_imu(sample(),s) && s.sensor_id==1 && s.gyro_valid() && s.control_valid());
  auto invalid=sample(); hw::write_float(invalid.payload.data()+16,std::numeric_limits<float>::quiet_NaN());
  CHECK(hw::decode_imu(invalid,s) && !s.orientation_valid());
  invalid=sample(); invalid.payload[0]=2; CHECK(!hw::decode_imu(invalid,s));
  invalid=sample(); invalid.payload[1]=0x80; CHECK(!hw::decode_imu(invalid,s));
  invalid=sample(); invalid.payload.resize(20); CHECK(!hw::decode_imu(invalid,s));
  CHECK(hw::decode_imu(sample(1,0,3),s) && !s.control_valid()); // Current firmware.

  hw::AdapterConfig config; config.allow_control=true; config.allow_nav=true;
  config.allow_friction=true; config.allow_shoot=true; config.max_error_deg=30;
  hw::CommandAdapter a(config);
  CHECK(a.accept(sample(),1000)); CHECK(!a.ready(1000));
  CHECK(a.accept(link(),1000)); CHECK(a.ready(1001));
  a.set_gimbal({-179*hw::PI/180,0.1,true,true},1001); a.set_nav({0.7,-0.2},1001);
  const auto aim=a.aim_command(1010); const auto& p=aim.payload;
  CHECK(aim.type==0x10 && aim.sequence==40001 && aim.boot_id==0x12345678);
  CHECK(hw::encode(aim).size()==38 && p.size()==24 && hw::read_u32(p.data())==0);
  CHECK(hw::read_u16(p.data()+4)==1 && hw::read_u16(p.data()+6)==10);
  CHECK(hw::read_u32(p.data()+8)==1000000 && hw::read_u16(p.data()+12)==100);
  CHECK(p[14]==15 && p[15]==0);
  CHECK(std::abs(hw::read_float(p.data()+16)-2*hw::PI/180)<1e-6);
  CHECK(std::abs(hw::read_float(p.data()+20)-0.2)<1e-6);
  const auto nav=a.nav_command(1010);
  CHECK(nav.type==0x11 && nav.payload[10]==1 && nav.payload[11]==0);
  CHECK(hw::read_float(nav.payload.data()+12)==float(0.7) && hw::read_float(nav.payload.data()+20)==0);
  CHECK(a.aim_command(1101).payload[14]==0 && a.nav_command(1101).payload[10]==0);
  CHECK(a.aim_command(1010,true).payload[14]==0);
  CHECK(!a.accept(sample(),1020)); // Duplicate cannot refresh freshness.
  CHECK(a.feedback_age(1020)==20);
  CHECK(a.accept(sample(11,1001000,3),1020));
  a.set_gimbal({0,0,true,true},1021); CHECK(a.aim_command(1022).payload[14]==0);
  CHECK(a.accept(sample(12,1002000,15),1030));
  a.set_gimbal({0,0,true,true},1031); CHECK(a.aim_command(1032).payload[14]==15);
  auto epoch=sample(13,1003000); hw::write_u16(epoch.payload.data()+2,2);
  CHECK(a.accept(epoch,1040)); CHECK(a.aim_command(1041).payload[14]==0);
  auto reboot=sample(0,1); reboot.boot_id=99;
  CHECK(a.accept(reboot,1050) && !a.ready(1050) && a.aim_command(1051).payload[14]==0);
  auto newlink=link(0,65535); newlink.boot_id=99;
  CHECK(a.accept(newlink,1050)); CHECK(a.aim_command(1051).sequence==0);
  a.disconnect(); CHECK(!a.ready(1052));
  CHECK(a.accept(reboot,1060) && a.accept(newlink,1060)); CHECK(a.aim_command(1061).payload[14]==0);
  hw::CommandAdapter defaults;
  defaults.accept(sample(),1000); defaults.accept(link(),1000);
  defaults.set_gimbal({0,0,true,true},1001); defaults.set_nav({1,1},1001);
  CHECK(defaults.aim_command(1002).payload[14]==0 && !defaults.nav_enabled());
  CHECK(defaults.nav_command(1002).payload[10]==0);
  hw::CommandAdapter repeated(config);
  repeated.accept(sample(),1000); repeated.accept(link(),1000);
  CHECK(!repeated.accept(sample(11),1050) && repeated.feedback_age(1050)==50);
  auto invalid_repeat=sample(12);
  hw::write_float(invalid_repeat.payload.data()+16,std::numeric_limits<float>::quiet_NaN());
  CHECK(!repeated.accept(invalid_repeat,1060));
  repeated.set_gimbal({0,0,true,true},1061);
  CHECK(repeated.aim_command(1062).payload[14]==0);
  CHECK(hw::newer(0,65535) && !hw::newer(65535,0) && !hw::newer(1,1) && !hw::newer(32768,0));
  std::cout<<"PASS: V2 framing/CRC, fragmented/noisy stream, radian offsets, exact reference, permissions, expiry, boot/epoch/sequence\n";
}
