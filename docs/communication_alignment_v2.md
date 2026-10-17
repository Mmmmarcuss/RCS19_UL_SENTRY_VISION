# V2 通信对齐

以本地 `Hohokam_SentinelRobot1009` 的 `8caff9c`（2026-10-15）为准，2026-10-16 完成上位机适配。下位机源码未修改。

**默认只接收。** `read_only: true` 使用只读串口，不创建发送定时器，退出不发停止帧。所有参数只允许启动时设置。即使打开其他许可，只接收模式也不发送。

**当前固件不能用于云台闭环。** 两路 IMU 的 `SAMPLE_TIME_VALID`、`AXES_VERIFIED` 仍为 0；云台执行宏也是 0。上位机保留原始诊断数据，阻止未确认数据进入云台控制和有效姿态接口。导航宏实际为 1，不能把整份固件当作“所有执行器只读”。

## 线上帧

921600、8N1，小端。统一外壳如下，CRC16/MCRF4XX 初值 FFFF、反射多项式 8408，无最终异或，覆盖帧头和负载，校验值低字节先发。

```text
A6 6A | version=2:u8 | type:u8 | payload_len:u16 | sequence:u16 | boot_id:u32 | payload | crc16:u16
```

| 方向 | 类型 | 负载 / 总长 | 内容 |
|---|---|---|---|
| 下→上 | 01 | 32 / 46 B | sensor_id、flags、epoch、sample_time_us、roll/pitch/yaw、gx/gy/gz |
| 下→上 | 02 | 20 / 34 B | sensor_id、flags、epoch、sample_time_us、roll/pitch/yaw |
| 下→上 | 81 | 12 / 26 B | host_session、last_aim_seq、state、last_reject、aim_age_ms、reserved |
| 上→下 | 10 | 24 / 38 B | host_session、参考 epoch/seq/time、有效期、flags、reserved、yaw/pitch_error |
| 上→下 | 11 | 24 / 38 B | host_session、ref_mcu_time_us、有效期、flags、reserved、vx/vy/wz |

角度和误差在线上使用 **rad**，角速度 **rad/s**，速度 **m/s**。`host_session` 和保留字段发 0。不使用会话握手、ACK 或时间同步消息，不兼容旧 COMMAND40/STATE32。当前实际代码包含 11 导航帧，尽管下位机精简文档只列了四种消息。

云台指令的参考四元组原样取自最近接收的 01：`boot_id、epoch、sequence、sample_time_us`。ROS 绝对目标角减去该样本姿态得到误差，yaw 处理 ±π 回绕；误差限幅参数仍用度配置。禁止把 IMU 的 gx/gy/gz 当作欧拉角导数。

## 启动与接口

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rm_interfaces rm_hw_bridge rm_autoaim
source install/setup.bash
ros2 launch rm_hw_bridge hw_bridge.launch.py serial_device:=/dev/ttyUSB0
```

上述是通信/自瞄最小构建。`GimbalStatus` 新增字段后，旧二进制不能混用：整车启动前还需重新构建依赖 `rm_interfaces` 的视觉、决策和行为树等包，导航依赖需事先安装齐全。

顶层 `sentry_bringup.launch.py` 也提供 `read_only、allow_control、allow_nav、allow_friction、allow_shoot` 五个启动参数，默认分别为 true、false、false、false、false。

| 话题 | 含义 |
|---|---|
| `/hw_bridge/imu_telemetry` | 两路 IMU 原值及 sensor_id、flags、boot_id、epoch、seq、MCU 时间；包含字段有限值检查结果 |
| `/imu/data` | 云台 IMU，弧度 / rad/s |
| `/chassis/imu/data` | 底盘 IMU；角速度、加速度不提供 |
| `/gimbal_status` | 保留既有度数接口；新增 pose_valid；失效、轴向未核验或超时时角度为 NaN |
| `/hw_bridge/link_status` | 串口状态、收发计数、数据年龄、参考样本与下位机拒绝原因 |

ROS `header.stamp` 全部为**上位机接收时刻**。`sample_time_us` 单独保存原值，未建立 MCU→ROS 时钟映射。轴向未核验或姿态无效时，标准 IMU 消息的 `orientation_covariance[0]=-1`；角速度同样检查有效标志、轴向及有限值。自瞄忽略这种无效姿态。原始诊断话题仍可查看角度和角速度。

两块 IMU 的姿态参考系可能不同，不能直接相减当成关节角。`publish_joint_states` 默认 false；仅为已核实旧接口语义的使用者保留兼容选项。轴向核验后，桥接按 `Rz(yaw) Ry(pitch) Rx(roll)` 转四元数；这要求电控输出符合该约定及实际 IMU 安装坐标系，不能单凭 flags 位推测 ENU/NWU/NED 或交换 roll/pitch。

本版没有敌方颜色、裁判信息、模式、弹速、实际底盘速度，相关可用标志置 false，颜色置 255，未知浮点状态置 NaN。检测颜色使用视觉配置的 fallback。gx/gy/gz 仅发布到 IMU/原始诊断消息，兼容状态中的 Euler rates 维持不可用。

## 发送规则

- 默认不发。关闭只接收后，等待当前 boot 的 81 状态，用 `last_aim_seq+1` 起始发送云台帧；断连、重启或参考 epoch 改变清除旧指令。
- 云台必须显式 `allow_control=true`，且姿态、样本时间、轴向有效、参考和上层指令均未超时；否则发 flags=0、误差=0 的停止帧。摩擦轮、开火各需单独许可；开火必须同时允许摩擦轮。
- 导航必须单独 `allow_nav=true`；未启用时不发 11。启用后按新鲜 `/nav_cmd` 发送 vx/vy，超时发禁用零速度帧。当前下位机忽略 wz，桥接固定发 0，并提示不支持旋转。
- 默认发送 50 Hz，每次最多拼接两帧共 76 B，一次写出，适配下位机 100 B 接收快照。有效期默认 100 ms，限制为 1..150 ms。
- 上述仅是协议和本机门槛，不代表电控许可、机械映射和实车闭环已验证。

## 下位机待处理

1. **IMU 时间来源错误**：当前两路 `sample_time_us` 实际取自上位机 UART4 的最近接收时刻；仅接收时通常一直为 0。应分别来自两路真实 IMU 样本，并完成采样时钟标定后再置 `SAMPLE_TIME_VALID`。
2. **轴与安装关系**：分别核验 roll/pitch/yaw 和 gx/gy/gz、正方向、欧拉角约定和参考系，再置 `AXES_VERIFIED`。这版驱动已改过 roll/pitch 顺序，旧实机验证不能直接沿用。
3. **重启与序号**：当前 boot_id 算法不保证跨复位唯一、epoch 恒为 1。导航没有回传最后接受的序号，上位机进程重启后 NAV 序号可能被拒绝；不能声称导航能无条件自动恢复。联调前需复位下位机或由电控补齐序号恢复机制。
4. **执行代码**：云台控制关闭分支之外存在 `xuc::WrapPi` 大小写问题，放行前需修正编译和核验执行路径。上位机未修改该下位机文件。
5. 81 中 `AIM_FRESH` 不是完整的租期判据，不能单靠该位判断命令未过期。当前无会话/时钟同步，导航按 MCU 收包后的本地租期执行。

## 离线验证

已使用 WSL Ubuntu 22.04 / ROS 2 Humble 完成消息、串口桥、自瞄构建，以及下列测试：

```bash
cmake -S src/rm_hw_bridge -B .cache/v2_protocol_tests -DPROTOCOL_TESTS_ONLY=ON
cmake --build .cache/v2_protocol_tests
ctest --test-dir .cache/v2_protocol_tests --output-on-failure
python3 src/rm_hw_bridge/tests/teammate_source_check.py
# 加载新编译的 ROS 环境后：
python3 src/rm_hw_bridge/tests/teammate_pty_test.py
# 独立检查消息消费者（无需完整 Nav2）：
cmake -S src/rm_hw_bridge/tests/consumer_check -B .cache/v2_consumer_check
cmake --build .cache/v2_consumer_check
ctest --test-dir .cache/v2_consumer_check --output-on-failure
```

清理后项目不保留构建目录及详细日志。测试脚本可用 `RM_HW_BRIDGE_TEST_ROOT` 将产物放到工程外；虚拟串口测试也支持 `BRIDGE_TEST_BINARY` 指定刚编译的节点，或自动使用项目 `install/` 中的节点。此次重建产物和原始日志统一存放在工作区 `archives/project_cleanup_20261016_85f77d7b/validation/`。

- 协议单测覆盖 CRC、噪声/拆包/粘包、字段单位、精确参考、权限、超时、boot/epoch 和序号回绕。
- 下位机源码测试直接编译未改动的 `xuc.cpp/xuc.h/CRC.cpp/CRC.h`，只替换 UART、时钟、IMU 和 RTOS 外设，验证真实 Encode/Decode 的双向兼容。测试中单独模拟“标定已完成”标志来验证控制封包；没有修改真实固件。
- ROS 虚拟串口验证双 IMU 话题、错误标志阻断、弧度误差限幅、摩擦轮/发射许可、陈旧指令清零、boot 改变、断连恢复，以及只接收模式启动/运行/退出均零字节发送。参数运行中修改被拒绝。
- 单独编译了最终指令合成节点与裁判状态消费者，并通过缺失裁判信息的回归检查。测试记录见 [验证结果](validation_communication_v2/results.md)。

当前 WSL 未安装 `nav2_msgs`，完整导航/决策包构建未完成；本轮仅核验通信改动及相关消费者。未部署小电脑、未连接实车串口、未验证电机执行或闭环效果。
