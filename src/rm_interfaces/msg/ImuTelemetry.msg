# V2 原始诊断数据；header.stamp 是上位机接收时刻，不是 MCU 采样时刻。
# 参考系、安装方向未确认前，不作为导航/弹道姿态输入。
std_msgs/Header header
uint8 sensor_id              # 1=云台，2=底盘
uint8 flags                  # bit0姿态有效 bit1角速度有效 bit2样本时间有效 bit3轴向已核验
uint16 imu_epoch
uint16 sequence
uint32 boot_id
uint32 sample_time_us        # MCU 原值，允许回绕；当前固件并非真实 IMU 采样时间
float32 roll                 # rad
float32 pitch
float32 yaw
float32 gx                   # IMU 本体轴 rad/s，底盘帧不提供
float32 gy
float32 gz
bool orientation_valid       # flags + 有限值检查，不等于轴向/时间已核验
bool gyro_valid
