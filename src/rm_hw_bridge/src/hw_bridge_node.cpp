#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/gimbal_status.hpp"
#include "rm_interfaces/msg/imu_telemetry.hpp"
#include "rm_interfaces/msg/nav_cmd.hpp"
#include "rm_hw_bridge/command_adapter.hpp"
#include "rm_hw_bridge/serial_port.hpp"

namespace hw = rm_hw_bridge;
static uint64_t steady_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ParsedFrame { hw::Frame frame; int64_t ros_ns; uint64_t steady_stamp; };

class HwBridgeNode : public rclcpp::Node {
 public:
  HwBridgeNode() : Node("rm_hw_bridge") {
    const auto device = parameter<std::string>("serial_device", "/dev/ttyUSB0");
    const auto baud = parameter<int>("baudrate", 921600);
    if (baud != 921600) throw std::invalid_argument("Firmware V2 requires baudrate=921600");
    read_only_ = parameter<bool>("read_only", true);
    frame_id_ = parameter<std::string>("frame_id", "gimbal_imu_link");
    chassis_frame_id_ = parameter<std::string>("chassis_frame_id", "chassis_imu_link");
    publish_joint_states_ = parameter<bool>("publish_joint_states", false);
    if (!parameter<bool>("require_crc_check", true)) throw std::invalid_argument("V2 requires CRC checking");
    hw::AdapterConfig config;
    config.allow_control = parameter<bool>("allow_control", false);
    config.allow_nav = parameter<bool>("allow_nav", false);
    config.allow_friction = parameter<bool>("allow_friction", false);
    config.allow_shoot = parameter<bool>("allow_shoot", false);
    config.yaw_error_sign = parameter<double>("yaw_error_sign", 1.0);
    config.pitch_error_sign = parameter<double>("pitch_error_sign", 1.0);
    config.correction_scale = parameter<double>("correction_scale", 1.0);
    config.max_error_deg = parameter<double>("max_error_deg", 5.0);
    config.command_timeout_ms = timeout_parameter("command_timeout_ms", 100);
    config.nav_timeout_ms = timeout_parameter("nav_timeout_ms", 100);
    config.feedback_timeout_ms = timeout_parameter("feedback_timeout_ms", 100);
    feedback_timeout_ = config.feedback_timeout_ms;
    const int validity = parameter<int>("valid_for_ms", 100);
    if (validity<1 || validity>150) throw std::invalid_argument("valid_for_ms must be 1..150");
    config.valid_for_ms = uint16_t(validity);
    if (std::abs(config.yaw_error_sign)!=1 || std::abs(config.pitch_error_sign)!=1 ||
        !std::isfinite(config.correction_scale) || config.correction_scale<=0 ||
        !std::isfinite(config.max_error_deg) || config.max_error_deg<=0)
      throw std::invalid_argument("Invalid correction sign/scale/limit");
    if (config.allow_shoot && !config.allow_friction)
      throw std::invalid_argument("allow_shoot requires allow_friction");
    adapter_ = hw::CommandAdapter(config);
    const int tx_hz = parameter<int>("tx_rate_hz", 50);
    if (tx_hz<10 || tx_hz>50) throw std::invalid_argument("tx_rate_hz must be 10..50");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    chassis_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/chassis/imu/data", rclcpp::SensorDataQoS());
    telemetry_pub_ = create_publisher<rm_interfaces::msg::ImuTelemetry>("/hw_bridge/imu_telemetry", rclcpp::SensorDataQoS());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<rm_interfaces::msg::GimbalStatus>("/gimbal_status", rclcpp::SensorDataQoS());
    link_pub_ = create_publisher<std_msgs::msg::String>("/hw_bridge/link_status", 10);
    serial_ = std::make_unique<hw::SerialPort>(device, baud,
      [this](const uint8_t* data, size_t n) {
        std::lock_guard<std::mutex> lock(rx_mutex_);
        parser_.feed(data, n, [this](const hw::Frame& frame) {
          if (received_.size()>=64) { received_.pop(); ++queue_drops_; }
          received_.push({frame, now().nanoseconds(), steady_ms()});
        }, steady_ms());
      }, get_logger(), read_only_);
    gimbal_sub_ = create_subscription<rm_interfaces::msg::GimbalCmd>(
      "/gimbal_cmd", rclcpp::SensorDataQoS(), [this](rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
        adapter_.set_gimbal({msg->target_yaw,msg->target_pitch,msg->mode!=0,msg->fire_control==1},steady_ms());
        if (msg->rule_action_type || msg->spin_mode || msg->supercap_mode || msg->revive_cmd ||
            msg->remote_ammo_req_inc || msg->remote_hp_req_inc)
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"V2 does not carry tactical/referee actions");
      });
    nav_sub_ = create_subscription<rm_interfaces::msg::NavCmd>(
      "/nav_cmd",rclcpp::SensorDataQoS(),[this](rm_interfaces::msg::NavCmd::SharedPtr msg) {
        adapter_.set_nav({msg->linear_x,msg->linear_y},steady_ms());
        if (!std::isfinite(msg->angular_z) || std::abs(msg->angular_z)>1e-6)
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Firmware ignores NAV wz; transmitting wz=0");
      });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(5),[this] { publish_received(); });
    // In receive-only mode there is no TX timer, no write, and no exit STOP.
    if (!read_only_) tx_timer_ = create_wall_timer(std::chrono::microseconds(1000000/tx_hz),[this] { transmit(false); });
    RCLCPP_INFO(get_logger(),"V2 read_only=%d control=%d nav=%d friction=%d shoot=%d",
      read_only_,config.allow_control,config.allow_nav,config.allow_friction,config.allow_shoot);
  }
  ~HwBridgeNode() override {
    if (serial_) { if (!read_only_) transmit(true); serial_->shutdown(); }
  }
 private:
  template<class T> T parameter(const std::string& name,const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true; // Configuration changes require a deliberate restart.
    return declare_parameter<T>(name,value,descriptor);
  }
  uint64_t timeout_parameter(const char* name,int value) {
    const int timeout=parameter<int>(name,value);
    if (timeout<1 || timeout>1000) throw std::invalid_argument(std::string(name)+" must be 1..1000 ms");
    return uint64_t(timeout);
  }
  void transmit(bool stop) {
    const auto tick=steady_ms();
    if (read_only_ || !serial_->is_ok() || !adapter_.ready(tick)) return;
    auto bytes=hw::encode(adapter_.aim_command(tick,stop));
    if (adapter_.nav_enabled()) {
      const auto nav=hw::encode(adapter_.nav_command(tick,stop));
      bytes.insert(bytes.end(),nav.begin(),nav.end());
    }
    // One write, at most 76 bytes: fits the MCU's 100-byte RX snapshot.
    if (serial_->write(bytes.data(),bytes.size())) ++tx_count_;
  }
  void publish_received() {
    std::queue<ParsedFrame> batch;
    uint64_t valid,rejected,drops;
    {
      std::lock_guard<std::mutex> lock(rx_mutex_); received_.swap(batch);
      valid=parser_.valid_frames(); rejected=parser_.rejected_frames(); drops=queue_drops_;
    }
    const auto tick=steady_ms();
    if (!serial_->is_ok() || (last_rx_ms_ && tick-last_rx_ms_>=feedback_timeout_)) adapter_.disconnect();
    while (!batch.empty()) {
      const auto pf=batch.front(); batch.pop();
      if (!serial_->is_ok() || tick<pf.steady_stamp || tick-pf.steady_stamp>=feedback_timeout_ ||
          !adapter_.accept(pf.frame,pf.steady_stamp)) continue;
      last_rx_ms_=pf.steady_stamp;
      hw::ImuSample sample;
      if (hw::decode_imu(pf.frame,sample)) {
        publish_imu(sample,rclcpp::Time(pf.ros_ns));
        if (sample.sensor_id==1) publish_status(pf.ros_ns,tick);
      }
    }
    if (tick-last_health_ms_<100) return;
    last_health_ms_=tick;
    if (!adapter_.feedback_fresh(tick)) publish_status(now().nanoseconds(),tick);
    std_msgs::msg::String link;
    const auto& s=adapter_.feedback(); const auto& l=adapter_.link();
    std::ostringstream out;
    out<<"serial_open="<<serial_->is_ok()<<" read_only="<<read_only_<<" tx_batches="<<tx_count_
       <<" rx_frames="<<valid<<" rejected="<<rejected<<" queue_drops="<<drops
       <<" boot_id="<<s.boot_id<<" epoch="<<s.epoch<<" imu_seq="<<s.sequence
       <<" sample_time_us="<<s.sample_time_us<<" imu_flags="<<unsigned(s.flags)
       <<" receipt_age_ms="<<adapter_.feedback_age(tick)<<" link_fresh="<<adapter_.ready(tick)
       <<" last_aim_seq="<<l.last_aim_seq<<" mcu_state="<<unsigned(l.state)
       <<" last_reject="<<unsigned(l.last_reject)<<" aim_age_ms="<<l.aim_age_ms;
    link.data=out.str(); link_pub_->publish(link);
    RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"%s",link.data.c_str());
  }
  void publish_imu(const hw::ImuSample& s,const rclcpp::Time& stamp) {
    rm_interfaces::msg::ImuTelemetry raw;
    raw.header.stamp=stamp; raw.header.frame_id=s.sensor_id==1?frame_id_:chassis_frame_id_;
    raw.sensor_id=s.sensor_id; raw.flags=s.flags; raw.imu_epoch=s.epoch;
    raw.sequence=s.sequence; raw.boot_id=s.boot_id; raw.sample_time_us=s.sample_time_us;
    raw.roll=s.roll; raw.pitch=s.pitch; raw.yaw=s.yaw;
    raw.gx=s.gx; raw.gy=s.gy; raw.gz=s.gz;
    raw.orientation_valid=s.orientation_valid(); raw.gyro_valid=s.gyro_valid();
    telemetry_pub_->publish(raw);
    sensor_msgs::msg::Imu imu;
    // ROS stamp is receipt time. MCU clock is kept separately; there is no time sync.
    imu.header=raw.header;
    const bool pose_valid=s.orientation_valid() && (s.flags&hw::AXES_VERIFIED);
    if (pose_valid) {
      const double cy=std::cos(s.yaw/2),sy=std::sin(s.yaw/2),cp=std::cos(s.pitch/2),sp=std::sin(s.pitch/2);
      const double cr=std::cos(s.roll/2),sr=std::sin(s.roll/2);
      imu.orientation.w=cr*cp*cy+sr*sp*sy; imu.orientation.x=sr*cp*cy-cr*sp*sy;
      imu.orientation.y=cr*sp*cy+sr*cp*sy; imu.orientation.z=cr*cp*sy-sr*sp*cy;
    } else { imu.orientation.w=1; imu.orientation_covariance[0]=-1; }
    if (s.gyro_valid() && (s.flags&hw::AXES_VERIFIED)) {
      imu.angular_velocity.x=s.gx; imu.angular_velocity.y=s.gy; imu.angular_velocity.z=s.gz;
    } else imu.angular_velocity_covariance[0]=-1;
    imu.linear_acceleration_covariance[0]=-1;
    if (s.sensor_id==1) {
      imu_pub_->publish(imu);
      if (pose_valid && publish_joint_states_) {
        sensor_msgs::msg::JointState js; js.header=imu.header;
        js.name={"gimbal_yaw","gimbal_pitch"}; js.position={s.yaw,s.pitch}; joint_pub_->publish(js);
      }
    } else chassis_imu_pub_->publish(imu);
  }
  void publish_status(int64_t stamp,uint64_t tick) {
    rm_interfaces::msg::GimbalStatus status;
    status.header.stamp=rclcpp::Time(stamp); status.header.frame_id=frame_id_;
    const auto& s=adapter_.feedback();
    const float unavailable=std::numeric_limits<float>::quiet_NaN();
    status.pose_valid=adapter_.feedback_fresh(tick) && s.orientation_valid() && (s.flags&hw::AXES_VERIFIED);
    status.gimbal_yaw=status.pose_valid?float(s.yaw*180/hw::PI):unavailable;
    status.gimbal_pitch=status.pose_valid?float(s.pitch*180/hw::PI):unavailable;
    status.gimbal_roll=status.pose_valid?float(s.roll*180/hw::PI):unavailable;
    // Body gyro xyz are not Euler angle derivatives. Keep them in Imu/ImuTelemetry.
    status.gimbal_yaw_rate=status.gimbal_pitch_rate=status.gimbal_roll_rate=unavailable;
    status.bullet_speed=status.chassis_vx=status.chassis_vy=status.chassis_wz=unavailable;
    status.chassis_power=status.remain_energy=unavailable;
    status.referee_data_valid=status.mode_valid=status.ballistic_data_valid=status.rates_valid=false;
    status.detect_color=255;
    status_pub_->publish(status);
  }
  bool read_only_=true;
  bool publish_joint_states_=false;
  std::string frame_id_,chassis_frame_id_;
  hw::CommandAdapter adapter_; hw::StateParser parser_;
  std::mutex rx_mutex_; std::queue<ParsedFrame> received_;
  uint64_t tx_count_=0,queue_drops_=0,last_rx_ms_=0,last_health_ms_=0,feedback_timeout_=100;
  std::unique_ptr<hw::SerialPort> serial_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<rm_interfaces::msg::NavCmd>::SharedPtr nav_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_,chassis_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::ImuTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_,tx_timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<HwBridgeNode>(); rclcpp::spin(node); node.reset(); rclcpp::shutdown(); return 0;
}
