# 上位机通信对齐（2026-10-09）

已在 `E:/RM/RM27_UL_backup/RCS_SENTRY_VISION_20261009` 适配队友仓库提交
`6a3fc9ea454de055be92e0b6cd493265eb4c146b`。只修改上位机及其说明和验证代码，下位机源码未改。
详细下位机字段表见 [源码检查报告](teammate_communication_20261009.md)。

## 实现

- 串口：921600、8N1、无流控；使用原始二进制模式，完整写入一帧，掉线自动重连。
- 下行：固定40字节、0xA5头、CRC16覆盖前38字节。自瞄和导航合成同一帧，默认50Hz。
- 上行：固定32字节、0x5A头、CRC16覆盖前30字节。支持噪声、半帧、粘包后重新找帧头。
- 数值：小端float32/uint16；CRC初值FFFF、反射多项式8408、无最终异或。
- `/gimbal_cmd` 保持绝对目标角（rad）；桥接根据最新反馈计算剩余误差，yaw取最短角差，转换成度发送。
  下行pitch/yaw放反馈的当前角度（度），yaw_diff/pitch_diff放纠偏量（度）。
- `/nav_cmd.linear_x/linear_y` 并入偏移30/34。保留字节、distance、v_y暂时填零。
- 上行roll/pitch/yaw按度解码。CH010 HI91的角度是度，队友驱动和Encode均未转换；
  原协议文档标成弧度与源码/厂家手册不一致，本次复查已纠正。
  `/imu/data`四元数和`/joint_states.position`转换后使用弧度；
  `/gimbal_status`保留原项目的度单位。JointState沿用IMU角输出，并不是独立电机编码器相对底盘角。
- 回传rx_total/rx_count进入`/gimbal_status`；`/hw_bridge/link_status`和控制台打印发送数、累计接收数和增量。
- 超时：云台命令150ms无更新则纠偏和开火清零，导航250ms无更新则速度清零，
  姿态反馈300ms无更新则停止云台纠偏和开火。导航新鲜度独立检查。
- STATE32没有裁判信息、模式、弹速或角速度。新增有效性字段；缺失浮点状态用NaN表示，
  不把缺失的整数零值当作真实裁判状态。视觉直接使用detect_color选择敌方颜色。
  决策不把该回包当作完整裁判链路，命令合成器缺少裁判依据时禁止发射。
  自瞄弹道使用已有`bullet_speed_default`配置，当前是24m/s，应按实测设置。

主要文件：`src/rm_hw_bridge`、`src/rm_interfaces/msg/GimbalStatus.msg`、
`src/rm_vision/src/vision_detector_node.cpp`、`src/sentry_bt/src/referee_interface.cpp`、
`src/rm_sentry_decision/src/sentry_command_mux_node.cpp`。启动默认波特率和README示例已同步。
修改前文件备份位于 `.cache/communication_before_40_32`。

## 编译与基础链路测试

以下命令在机器人Linux/ROS2 Humble终端执行。修改了ROS消息定义，不能继续混用旧的消息库和节点二进制。
通信单独测试只需要编译两个包；完整自瞄/决策启动需在依赖齐全的环境重新编译所有消费者。

```bash
cd /path/to/RCS_SENTRY_VISION_20261009
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_interfaces rm_hw_bridge
source install/setup.bash

# 单独开桥，不同时启动其他占用同一串口的节点。
# allow_control=false使纠偏、速度和开火均为零，仍发送合法40字节帧供计数验证。
ros2 run rm_hw_bridge hw_bridge_node --ros-args \
  --params-file src/rm_hw_bridge/config/params.yaml \
  -p serial_device:=/dev/ttyUSB0 -p allow_control:=false
```

另一个终端同样source后观察：

```bash
ros2 topic echo /hw_bridge/link_status
ros2 topic echo /gimbal_status
```

在队友当前固件中，必须由遥控器切到AUTOAIM才发送约10Hz反馈；上位机没有切换该模式的线协议字段。
`rx_count`持续增加说明下位机接受了合法下行；同时收到合法STATE32说明回传方向也打通。
这是累计接受统计，不是某条指令的逐帧ACK，更不证明执行器已完成动作。
计数增量按uint16取模，重启与回绕无法仅凭这两个字段区分。
`rx_total`只统计被取出的队列项，不能用于计算准确的线路丢帧率。

UART4实际配置PA0 TX、PA1 RX；源码初始化行的旧尾注PC10/PC11不可靠。
接线以实际板卡和烧录固件为准，TX/RX交叉连接并共地。
**下位机AUTOAIM收到有效帧就会启动摩擦轮，即使所有动作字段为零。**
`allow_control=false`或`allow_shoot=false`不能通过现有协议关闭摩擦轮。

完整环境重新编译：

```bash
colcon build --packages-up-to rm_bringup rm_autoaim rm_vision rm_sentry_decision sentry_bt
source install/setup.bash
ros2 launch rm_hw_bridge hw_bridge.launch.py serial_device:=/dev/ttyUSB0
```

## 控制联调与功能边界

`src/rm_hw_bridge/config/params.yaml`默认`allow_control=true`、`allow_shoot=false`。
`yaw_error_sign`、`pitch_error_sign`默认+1，可设-1以匹配机构纠偏方向。
`correction_scale=1.0`，`max_error_deg=5.0`是输出误差限幅，不是电机总行程限位。
`rm_autoaim`保持`output_relative_command=false`，不要把相对误差再当作绝对角送进桥接。

下位机每约5ms把保存的误差乘0.5并累加到目标位置，直到新报文覆盖；
因此一帧误差可能执行多次，而反馈只有约100ms一次。当前适配能正确传输和换算，
不能保证这个控制环稳定。实机先验证角度单位、正方向和小误差响应，再调整scale/限幅；
下位机自身约0.3度死区也会影响缩小误差后的响应。

另一个待核实点：队友`imu.cpp`将HI91偏移48（厂家roll）存入angle.pitch，
偏移52（厂家pitch）存入angle.roll。上位机保留STATE32字段对应关系，未擅自交换控制轴。
这可能与安装方向有关，也可能是驱动错误；必须用实机单轴旋转验证。
在确认安装轴、正方向及欧拉角约定之前，不能把目前的IMU四元数当作已标定的标准ROS姿态。
厂家字段单位依据：[HiPNUC指令与编程手册，HI91](https://download.hipnuc.com/legacy/imu_cum_cn_150_170.pdf)。

下位机IMU校验还有明确的表达式问题：`imu.cpp:55`写成
`rxData[3] << 8 + rxData[2]`及`rxData[5] << 8 + rxData[4]`。
C++按`a << (8+b)`计算，这不是长度/CRC的16位小端拼接，且移位数可能越界。
长度应按`(uint16_t(rxData[3]) << 8) | rxData[2]`组合，CRC同理，
并检查实际接收长度后再读取。此问题在下位机范围，按用户要求未修改。
即使收到合法STATE32，也不能据此断言内部IMU姿态已正确更新。

当前协议不支持底盘angular_z、导航到达状态、独立模式指令、战术/姿态切换、远程补弹/回血/复活等扩展动作；
桥接不发送这些字段，对非零旋转或部分战术请求输出提示。
比赛决策依赖的HP/热量/弹量/比赛状态也不在STATE32中，所以该版本不能靠这条链路完成完整比赛决策和发射许可。
fireadvice为持续电平，不能提供可靠单发去重或执行确认。

下位机接收仍仅识别一次UART IDLE缓冲中的第一个40字节报文，队列深度为1；
上位机用独立50Hz完整帧发送以留出间隔，但无法修复USB串口突发合包或下位机接收长度处理的限制。
反馈没有采样时间戳，ROS时间戳是上位机收到该包的时间，不能当作精确IMU采样时间。

## 验证

验证环境：本机WSL Ubuntu22.04、GCC11、ROS2 Humble；未接机器人、未烧录下位机。

- `rm_interfaces`消息生成、`rm_hw_bridge`和`rm_autoaim`编译安装成功。
- 协议CTest通过：字段位置、CRC、错误帧/NaN拒绝、所有分割点、噪声/粘包恢复、
  合成速度与角度、±180度跨界、禁火和超时、uint16回绕。
- 角度单位修正后的实际ROS节点虚拟串口测试通过，收到249帧COMMAND40；验证STATE32解析、
  角度单位、IMU四元数、接收计数回绕、开火位、保留位和停止发布指令后清零。
  此测试仅在虚拟串口中临时打开allow_shoot以检查位编码，默认配置仍禁火。
- 直接提取队友实际Encode/Decode、TxPacket和CRC源码进行主机交叉测试通过：
  1000帧上位机数据被Decode接受，错误CRC/帧头被拒绝，Encode回包可被上位机解码。
  UART/队列/IMU使用测试替身，未编译完整ARM固件或验证DMA调度。
- 相关包完整构建受当前WSL缺少Nav2/OpenVINO/BehaviorTree依赖限制。
  命令合成器和裁判接口源码已独立编译通过；测试确认无效裁判状态不会进入比赛模式，
  原有完整状态仍可正常使用。视觉颜色适配已检查源码，因OpenVINO缺失未完成视觉包编译。
  编译和测试日志记录在 `validation_communication_20261009`。

复现协议和虚拟串口测试：

```bash
cmake -S src/rm_hw_bridge -B .cache/teammate_protocol_tests -DPROTOCOL_TESTS_ONLY=ON
cmake --build .cache/teammate_protocol_tests
ctest --test-dir .cache/teammate_protocol_tests --output-on-failure

colcon build --packages-select rm_interfaces rm_hw_bridge \
  --build-base .cache/communication_build --install-base .cache/communication_install
source .cache/communication_install/setup.bash
python3 src/rm_hw_bridge/tests/teammate_pty_test.py
python3 src/rm_hw_bridge/tests/teammate_source_check.py /path/to/Hohokam_SentinelRobot1009
```
