#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/gimbal_status.hpp"
#include "rm_interfaces/msg/nav_cmd.hpp"
#include "rm_hw_bridge/command_adapter.hpp"
#include "rm_hw_bridge/serial_port.hpp"

namespace hw = rm_hw_bridge;
static uint64_t steady_ms() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ParsedFrame {
  hw::NucToVisionFrame state;
  int64_t ros_ns;
  uint64_t steady_stamp;
};

class HwBridgeNode : public rclcpp::Node {
 public:
  HwBridgeNode() : Node("rm_hw_bridge") {
    const auto device = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    const int baud = declare_parameter<int>("baudrate", 921600);
    if (baud != 921600) throw std::invalid_argument("Teammate firmware requires baudrate=921600");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    if (!declare_parameter<bool>("require_crc_check", true))
      throw std::invalid_argument("COMMAND40/STATE32 requires CRC checking");
    hw::AdapterConfig config;
    config.allow_control = declare_parameter<bool>("allow_control", true);
    config.allow_shoot = declare_parameter<bool>("allow_shoot", false);
    config.yaw_error_sign = declare_parameter<double>("yaw_error_sign", 1);
    config.pitch_error_sign = declare_parameter<double>("pitch_error_sign", 1);
    config.correction_scale = declare_parameter<double>("correction_scale", 1);
    config.max_error_deg = declare_parameter<double>("max_error_deg", 5);
    config.command_timeout_ms = timeout_parameter("command_timeout_ms", 150);
    config.nav_timeout_ms = timeout_parameter("nav_timeout_ms", 250);
    config.feedback_timeout_ms = timeout_parameter("feedback_timeout_ms", 300);
    if ((std::abs(config.yaw_error_sign) != 1) || (std::abs(config.pitch_error_sign) != 1) ||
        !std::isfinite(config.correction_scale) || config.correction_scale <= 0 ||
        !std::isfinite(config.max_error_deg) || config.max_error_deg <= 0)
      throw std::invalid_argument("Invalid correction sign/scale/limit");
    adapter_ = hw::CommandAdapter(config);
    const int tx_hz = declare_parameter<int>("tx_rate_hz", 50);
    if (tx_hz < 10 || tx_hz > 100) throw std::invalid_argument("tx_rate_hz must be 10..100");

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<rm_interfaces::msg::GimbalStatus>("/gimbal_status", rclcpp::SensorDataQoS());
    link_pub_ = create_publisher<std_msgs::msg::String>("/hw_bridge/link_status", 10);
    serial_ = std::make_unique<hw::SerialPort>(device, baud,
      [this](const uint8_t* data, size_t n) {
        parser_.feed(data, n, [this](const hw::NucToVisionFrame& state) {
          std::lock_guard<std::mutex> lock(rx_mutex_);
          if (received_.size() >= 64) received_.pop();
          received_.push({state, now().nanoseconds(), steady_ms()});
        });
      }, get_logger());
    gimbal_sub_ = create_subscription<rm_interfaces::msg::GimbalCmd>(
      "/gimbal_cmd", rclcpp::SensorDataQoS(),
      [this](rm_interfaces::msg::GimbalCmd::SharedPtr msg) {
        adapter_.set_gimbal({msg->target_yaw, msg->target_pitch, msg->mode != 0,
                            msg->fire_control == 1}, steady_ms());
        if (msg->rule_action_type || msg->spin_mode || msg->supercap_mode ||
            msg->revive_cmd || msg->remote_ammo_req_inc || msg->remote_hp_req_inc)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Teammate protocol does not carry tactical/referee action fields");
      });
    nav_sub_ = create_subscription<rm_interfaces::msg::NavCmd>(
      "/nav_cmd", rclcpp::SensorDataQoS(),
      [this](rm_interfaces::msg::NavCmd::SharedPtr msg) {
        adapter_.set_nav({msg->linear_x, msg->linear_y}, steady_ms());
        if (std::isfinite(msg->angular_z) && std::abs(msg->angular_z) > 1e-6)
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Teammate COMMAND40 has no chassis angular_z field; rotation is not transmitted");
      });
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { publish_received(); });
    tx_timer_ = create_wall_timer(std::chrono::microseconds(1000000/tx_hz), [this] {
      const auto bytes = hw::encode(adapter_.command(steady_ms()));
      if (serial_->write(bytes.data(), bytes.size())) ++tx_count_;
    });
    RCLCPP_INFO(get_logger(), "COMMAND40/STATE32, TX=%dHz, control=%d, shoot=%d",
      tx_hz, config.allow_control, config.allow_shoot);
  }
  ~HwBridgeNode() override {
    // One zero frame overwrites old correction/fire/speed. Firmware may still spin friction wheels.
    if (serial_) {
      const auto zero = hw::encode(hw::VisionToNucFrame{});
      serial_->write(zero.data(), zero.size());
      serial_->shutdown(); // Stop callbacks while node members are still alive.
    }
  }
 private:
  uint64_t timeout_parameter(const char* name, int value) {
    const int timeout = declare_parameter<int>(name, value);
    if (timeout <= 0) throw std::invalid_argument(std::string(name)+" must be positive");
    return uint64_t(timeout);
  }
  void publish_received() {
    std::queue<ParsedFrame> batch;
    { std::lock_guard<std::mutex> lock(rx_mutex_); received_.swap(batch); }
    while (!batch.empty()) {
      const auto pf = batch.front(); batch.pop();
      adapter_.set_feedback(pf.state, pf.steady_stamp);
      const rclcpp::Time stamp(pf.ros_ns);
      publish_pose(pf.state, stamp);
      publish_status(pf.state, stamp);
      const uint16_t total_delta = have_counters_ ? hw::counter_delta(pf.state.rx_total, previous_total_) : 0;
      const uint16_t valid_delta = have_counters_ ? hw::counter_delta(pf.state.rx_count, previous_valid_) : 0;
      previous_total_ = pf.state.rx_total; previous_valid_ = pf.state.rx_count; have_counters_ = true;
      std_msgs::msg::String link;
      std::ostringstream out;
      out << "tx_frames=" << tx_count_ << " rx_total=" << pf.state.rx_total
          << " rx_count=" << pf.state.rx_count << " delta_total=" << total_delta
          << " delta_valid=" << valid_delta;
      link.data = out.str(); link_pub_->publish(link);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "[LINK] %s", link.data.c_str());
    }
  }
  void publish_pose(const hw::NucToVisionFrame& s, const rclcpp::Time& stamp) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp; imu.header.frame_id = frame_id_;
    const double yaw = s.yaw*hw::PI/180, pitch = s.pitch*hw::PI/180, roll = s.roll*hw::PI/180;
    const double cy = std::cos(yaw/2), sy = std::sin(yaw/2);
    const double cp = std::cos(pitch/2), sp = std::sin(pitch/2);
    const double cr = std::cos(roll/2), sr = std::sin(roll/2);
    imu.orientation.w = cr*cp*cy+sr*sp*sy;
    imu.orientation.x = sr*cp*cy-cr*sp*sy;
    imu.orientation.y = cr*sp*cy+sr*cp*sy;
    imu.orientation.z = cr*cp*sy-sr*sp*cy;
    imu.angular_velocity_covariance[0] = -1; // Not present on this wire protocol.
    imu.linear_acceleration_covariance[0] = -1;
    imu_pub_->publish(imu);
    sensor_msgs::msg::JointState js;
    js.header = imu.header; js.name = {"gimbal_yaw", "gimbal_pitch"};
    js.position = {yaw, pitch}; // Retain existing ROS pose interface, radians.
    joint_pub_->publish(js);
  }
  void publish_status(const hw::NucToVisionFrame& s, const rclcpp::Time& stamp) {
    rm_interfaces::msg::GimbalStatus status;
    status.header.stamp = stamp; status.header.frame_id = frame_id_;
    // Both STATE32 and existing GimbalStatus consumers use degrees.
    status.gimbal_yaw = s.yaw;
    status.gimbal_pitch = s.pitch;
    status.gimbal_roll = s.roll;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    status.gimbal_yaw_rate = status.gimbal_pitch_rate = status.gimbal_roll_rate = unavailable;
    status.bullet_speed = unavailable;
    status.chassis_vx = status.chassis_vy = status.chassis_wz = unavailable;
    status.chassis_power = status.remain_energy = unavailable;
    status.referee_data_valid = status.mode_valid = status.ballistic_data_valid = status.rates_valid = false;
    status.detect_color = s.flags & 1; status.reset_tracker = (s.flags & 2) != 0;
    status.rx_total = s.rx_total; status.rx_count = s.rx_count;
    status_pub_->publish(status);
  }
  std::string frame_id_;
  hw::CommandAdapter adapter_;
  hw::StateParser parser_; // Only accessed by the serial reader thread.
  std::mutex rx_mutex_;
  std::queue<ParsedFrame> received_;
  uint64_t tx_count_ = 0;
  uint16_t previous_total_ = 0, previous_valid_ = 0;
  bool have_counters_ = false;
  std::unique_ptr<hw::SerialPort> serial_;
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<rm_interfaces::msg::NavCmd>::SharedPtr nav_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr link_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_, tx_timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HwBridgeNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
