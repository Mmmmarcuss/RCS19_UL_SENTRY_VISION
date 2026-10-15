# 软件验证结果

日期：2026-10-09。环境：WSL Ubuntu22.04、GCC11、ROS2 Humble。

| 项目 | 结果 |
|---|---|
| rm_interfaces消息生成 | 通过 |
| rm_hw_bridge编译安装 | 通过 |
| rm_autoaim编译安装 | 通过，见autoaim_build.log |
| COMMAND40/STATE32协议CTest | 1/1通过，见protocol_ctest.log |
| 实际ROS桥接节点PTY测试 | 单位修正后通过，249帧下行；见pty_review_result.log和pty_bridge.log |
| 队友实际Encode/Decode/CRC源码交叉测试 | 1000帧通过；见firmware_source_check.log |
| 命令合成器独立编译 | 通过，见consumer_build.log |
| 裁判接口独立编译与有效性测试 | 1/1通过，见consumer_ctest.log |
| 安装后的launch参数 | 通过；默认921600，见launch_args.log |
| git diff --check | 通过 |
| 队友仓库本地改动 | 无；HEAD为6a3fc9ea454de055be92e0b6cd493265eb4c146b |

rm_sentry_decision完整包构建缺少nav2_msgs；sentry_bt完整包缺少BehaviorTree；
rm_vision缺少OpenVINO，尚未完成视觉包编译。未接入真实串口或执行器。

PTY虚拟测试临时设allow_shoot=true以验证开火位编码；默认运行配置仍为false。
软件测试不能确认实机角度正方向、控制环稳定性或下位机DMA/IDLE接收的实际丢帧情况。

本轮复查修正了上行角度单位：此前按协议文档视为弧度，
实际CH010 HI91输出度，驱动和Encode未转换。测试数据也同步改为度，
验证下行当前姿态仍为度，ROS四元数和JointState仅转换一次到弧度。
