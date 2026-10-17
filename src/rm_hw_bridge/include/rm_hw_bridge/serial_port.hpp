#ifndef RM_HW_BRIDGE__SERIAL_PORT_HPP_
#define RM_HW_BRIDGE__SERIAL_PORT_HPP_
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/logging.hpp>

namespace rm_hw_bridge {
class SerialPort {
 public:
  using RxCallback = std::function<void(const uint8_t*, size_t)>;
  SerialPort(const std::string& device, int baud, RxCallback callback,
             const rclcpp::Logger& logger = rclcpp::get_logger("serial_port"), bool read_only = true)
    : device_(device), baud_(baud), read_only_(read_only), callback_(std::move(callback)), logger_(logger) {
    try_open();
    daemon_ = std::thread([this] {
      while (!quit_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (quit_) break;
        if (ok_) continue;
        if (reader_.joinable()) reader_.join();
        close_fd();
        if (!quit_) try_open();
      }
    });
  }
  ~SerialPort() { shutdown(); }
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;
  bool is_ok() const { return ok_; }
  // Complete one frame under one lock; never silently truncate a nonblocking write.
  bool write(const uint8_t* data, size_t size) {
    if (read_only_) return false;
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ < 0 || !ok_) return false;
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
    size_t sent = 0;
    while (sent < size && std::chrono::steady_clock::now() < deadline) {
      const ssize_t n = ::write(fd_, data+sent, size-sent);
      if (n > 0) { sent += size_t(n); continue; }
      if (n < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) break;
      fd_set ready; FD_ZERO(&ready); FD_SET(fd_, &ready);
      timeval timeout{0, 1000};
      const int result = select(fd_+1, nullptr, &ready, nullptr, &timeout);
      if (result < 0 && errno != EINTR) break;
    }
    if (sent == size) return true;
    ok_ = false;
    RCLCPP_WARN(logger_, "Serial frame write incomplete: %zu/%zu bytes; reopening", sent, size);
    return false;
  }
  void shutdown() {
    if (quit_.exchange(true)) return;
    if (daemon_.joinable()) daemon_.join();
    if (reader_.joinable()) reader_.join();
    close_fd();
  }
 private:
  static speed_t to_speed(int baud) {
    switch (baud) {
      case 115200: return B115200;
      case 460800: return B460800;
      case 921600: return B921600;
      default: throw std::invalid_argument("Unsupported serial baudrate");
    }
  }
  void try_open() {
    try {
      const speed_t speed = to_speed(baud_);
      int candidate = ::open(device_.c_str(), (read_only_ ? O_RDONLY : O_RDWR) | O_NOCTTY | O_NONBLOCK);
      if (candidate < 0) throw std::runtime_error("Cannot open " + device_);
      termios tty{};
      if (tcgetattr(candidate, &tty) != 0) {
        ::close(candidate); throw std::runtime_error("tcgetattr failed");
      }
      cfmakeraw(&tty); // Also disable CR/LF conversion, stripping and input parity processing.
      tty.c_cflag = (tty.c_cflag & ~(CSIZE | PARENB | CSTOPB | CRTSCTS)) | CS8 | CLOCAL | CREAD;
      tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
      cfsetospeed(&tty, speed); cfsetispeed(&tty, speed);
      if (tcsetattr(candidate, TCSANOW, &tty) != 0) {
        ::close(candidate); throw std::runtime_error("tcsetattr failed");
      }
      tcflush(candidate, TCIFLUSH);
      { std::lock_guard<std::mutex> lock(io_mutex_); fd_ = candidate; }
      ok_ = true;
      reader_ = std::thread([this, candidate] {
        uint8_t bytes[1024];
        while (!quit_ && ok_) {
          fd_set ready; FD_ZERO(&ready); FD_SET(candidate, &ready);
          timeval timeout{0, 10000};
          const int result = select(candidate+1, &ready, nullptr, nullptr, &timeout);
          if (result < 0 && errno == EINTR) continue;
          if (result == 0) continue;
          if (result < 0) { ok_ = false; break; }
          const ssize_t n = ::read(candidate, bytes, sizeof(bytes));
          if (n > 0) callback_(bytes, size_t(n));
          else if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
          else { ok_ = false; break; }
        }
      });
      RCLCPP_INFO(logger_, "Serial opened: %s @ %d 8N1", device_.c_str(), baud_);
    } catch (const std::exception& e) {
      ok_ = false;
      RCLCPP_WARN(logger_, "%s (will retry)", e.what());
    }
  }
  void close_fd() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    ok_ = false;
  }
  std::string device_;
  int baud_, fd_ = -1;
  const bool read_only_;
  std::atomic<bool> ok_{false}, quit_{false};
  RxCallback callback_;
  rclcpp::Logger logger_;
  std::mutex io_mutex_;
  std::thread reader_, daemon_;
};
} // namespace rm_hw_bridge
#endif
