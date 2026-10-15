# 队友下位机通信检查（2026-10-09）

后续上位机工作目录：`E:/RM/RM27_UL_backup/RCS_SENTRY_VISION_20261009`。

## 仓库快照与范围

- 来源：<https://github.com/Hohokamk/Sentinel_robot>
- 克隆位置：`E:/RM/RM27_UL_backup/Hohokam_SentinelRobot1009`。
- 分支：`main`。
- 当前提交：`6a3fc9ea454de055be92e0b6cd493265eb4c146b`。
- 提交日期：2026-10-09。
- 当前提交说明：`pitch限幅`。
- 首次检查提交：`d5e159f6769807f996127d88e4e3987bcdd7a066`，此次快进更新了两次提交。
- 本文记录下位机源码检查及对齐前的差异。指定上位机已完成适配，见 [通信对齐说明](communication_alignment_20261009.md)。
- 未编译、烧录或连接机器人；结论来自该提交的源码检查。

## 实际物理接口

`STM32F405/STM32F405.cpp:96` 初始化 `xuc.Init(&uart4, UART4, 921600)`。
`STM32F405/usart.cpp:266` 起的 UART4 分支实际配置：

- UART4，PA0 TX / PA1 RX，GPIO AF8。
- 921600、8N1，无硬件流控，16倍过采样。
- RX：DMA1 Stream2 / Channel4，100字节缓冲，普通DMA + UART IDLE。
- TX：DMA1 Stream4 / Channel4，普通DMA。

重要差异：初始化行的尾注仍写着 `COMMAND34 / STATE101, PC10 TX / PC11 RX`，
但实际通信代码现为40/32字节，实际UART4引脚是PA0/PA1。用户此前提供的协议文档写PC10/PC11，
也与这个提交的引脚配置不同。接线必须以实际烧录固件及板卡引脚为准。
本提交已经注释掉原来UART4的电源串口初始化。

## 下行：上位机到电控

依据 `STM32F405/xuc.h:10` 和 `STM32F405/xuc.cpp:24`：固定40字节，帧头0xA5。
float通过memcpy读取4字节，目标STM32F405按小端IEEE754 float32解释。

| 偏移 | 字节数 | 内容 | 解释 |
|---:|---:|---|---|
| 0 | 1 | header | 0xA5 |
| 1 | 4 | pitch | 度；解码后乘PI/180 |
| 5 | 4 | yaw | 度；解码后乘PI/180 |
| 9 | 4 | yaw_diff | 度；剩余偏航纠偏量 |
| 13 | 4 | pitch_diff | 度；剩余俯仰纠偏量 |
| 17 | 4 | distance | 已解析；本轮未确定单位或控制用途 |
| 21 | 1 | fireadvice | 只读bit0，映射到auto_shoot电平 |
| 22 | 3 | 保留 | 本提交不读取，也不回显 |
| 25 | 4 | v_y | 已解析；本轮未确定单位或控制用途 |
| 29 | 1 | 保留 | 本提交不读取 |
| 30 | 4 | speed_x | 导航前后速度；后续经K_NAV、NAV_MAX、DV_MAX处理 |
| 34 | 4 | speed_y | 导航左右速度；后续经K_NAV、NAV_MAX、DV_MAX处理 |
| 38 | 2 | CRC16 | 前38字节的CRC，低字节先发 |

导航和自瞄在同一个40字节报文里，没有独立NAVI报文解析。
车体速度单位m/s来自用户提供的协议约定，源码仅读取并通过K_NAV换算，实际尺度需联调确认。

## 上行：电控到上位机

依据 `STM32F405/xuc.h:11` 的packed TxPacket和 `STM32F405/xuc.cpp:54`：
上行由28字节扩展为32字节，帧头0x5A。新增static_assert(sizeof(TxPacket)==32)，
未增加各字段offsetof断言；本轮未做ARM编译验证。

| 偏移 | 字节数 | 内容 |
|---:|---:|---|
| 0 | 1 | 0x5A |
| 1 | 1 | bit0 detect_color；bit1 reset_tracker；bit2–7 reserved |
| 2 | 4 | roll |
| 6 | 4 | pitch |
| 10 | 4 | yaw |
| 14 | 4 | aim_x |
| 18 | 4 | aim_y |
| 22 | 4 | aim_z |
| 26 | 2 | rx_total：Decode成功取出队列项的累计次数，uint16小端 |
| 28 | 2 | rx_count：帧头、CRC通过并解析完成的累计帧数，uint16小端 |
| 30 | 2 | CRC16，覆盖前30字节，低字节先发 |

detect_color=己方颜色的取反；reset_tracker=0；reserved=15。
三个角度直接取GetAngleRoll/Pitch/Yaw，不在Encode中换单位。
当前枪口IMU选择CH010，解析其0x91角度字段并直接返回，Encode也未换单位。
2026-10-09复查更正：HI91原生欧拉角单位是度，上行实际应按度处理。
队友协议文档中的“弧度”与厂家手册不一致；此前上位机沿用该说明造成了错误换算，现已修正。
依据：[HiPNUC指令与编程手册，HI91](https://download.hipnuc.com/legacy/imu_cum_cn_150_170.pdf)。
此外imu.cpp把原始偏移48（厂家roll）存pitch、52（厂家pitch）存roll；安装轴和正方向仍需实机验证。
aim_x/y/z仍没有写入测试编号；本次确认信息使用新增的两个uint16字段，不复用aim_x/y/z。

新增回传计数已在代码中实现：Decode取队列成功后、检查帧头之前增加rx_total；
帧头和CRC通过并解析完毕后增加rx_count；Encode将两者复制到TxNuc再计算CRC发送。
因此收到合法32字节回包且rx_count持续增加，可以确认下位机持续接受合法下行数据。
这属于累计接收统计，不是具体指令的逐帧ACK，也不代表控制动作执行成功。

相邻回包的计数差应按uint16取模：`delta=(current-previous)&0xffff`。
200次/秒时约327.68秒回绕，重启也会使计数归零；当前帧没有boot_id，无法仅靠计数区分重启和回绕。

## 调度与控制语义

- `taskslist.cpp:143`：DecodeTask每轮调用xuc.Decode，随后vTaskDelay(5)。
- `taskslist.cpp:130`：ControlTask每轮rc.Update，随后vTaskDelay(5)。
- `taskslist.cpp:156`：ArmTask仅在AUTOAIM模式调用xuc.Encode，随后vTaskDelay(100)。
- configTICK_RATE_HZ=1000，因此5/100 tick名义对应5/100ms；实际周期还包含执行时间。
- 非AUTOAIM模式没有XUC上行回包，不能据此判定线路断开。
- `RC.cpp:47`：OnPC只在AUTOAIM模式应用上位机输入。
- rx_count变化重置50次控制循环的超时；未变化逐次递减；名义约250ms，不是时间戳计时。
- 超时清零底盘速度，关闭摩擦轮和auto_shoot；不会发送独立失联确认帧。
- yaw_diff/pitch_diff保存在成员变量中，控制任务每轮都使用，直到新报文覆盖或失联。
  这不是“一帧只执行一次”的增量事件：同一旧纠偏可能在多个控制周期重复累加。
- fireadvice是开火电平，没有单发请求ID、去重或执行确认。
- AUTOAIM中只要收到有效帧且未超时，OnPC会令openRub=true，即使fireadvice=0也会启动摩擦轮。
  因此零增量、零速度、禁火报文不能被理解为完全无执行器动作的心跳。

## CRC核对

下位机实际使用CRC.cpp的查表算法，初值0xFFFF、反射多项式0x8408、最终不异或。
Append/Verify覆盖帧长减2，CRC低字节在前。
已用离线脚本核对下位机wCRC_Table与上位机CRC16_TABLE：256项完全一致。
测试ASCII `123456789` 的CRC为0x6F91。因此主要差异不是CRC算法，而是帧布局及语义。

## 接收实现的限制

1. UART IDLE中断复制整个100字节缓冲到深度1队列，采用覆盖写。
2. 没有把实际接收长度（DMA剩余计数）交给Decode。
3. Decode只认缓冲第0字节的0xA5，再对前40字节校验；不搜索帧头、不累积半帧、不拆多帧。
4. 多帧进入同一次IDLE接收时只处理第一帧；突发接收可能覆盖队列或填满普通DMA缓冲。
5. 由于100字节缓冲不清空、也不检查实际长度，短接收可能混入上一包尾部数据；CRC降低误认概率，
   但不能代替长度和字节流处理。
6. DMATransmit会直接停DMA、改地址/长度再启动，没有忙检查和发送完成队列。
   当前约10Hz、短报文通常留有间隔，但这一实现不能作为任意发送频率的可靠保证。
7. 没有有限值/范围过滤、报文序号、会话号或具体下行编号回显。
   rx_total/rx_count现在会回传；crc_err仍只有调试器可观察。
8. rx_total统计的是Decode取出的队列项，不是UART物理收到的帧数。
   队列被覆盖的项不会计入；一次队列项可能包含半帧或多帧。
   因此rx_total-rx_count是被取出但未接受的队列项数，不能直接当作精确的链路丢帧率。

## 对齐前：与指定上位机拷贝的差异

当前上位机通信入口是 `src/rm_hw_bridge`，不是此前RM27_UL里的XucSender。

| 项目 | 对齐前的上位机拷贝 | 队友下位机 |
|---|---|---|
| 波特率默认/配置 | 460800 | 921600 |
| 自瞄下行 | SP头，47字节 | 0xA5头，40字节 |
| 导航下行 | NAVI头，19字节，独立发送 | speed_x/y并入40字节，未接收angular_z |
| 上行 | SP头，145字节 | 0x5A头，32字节 |
| 上行姿态处理 | 将收到角度按度转弧度 | 实际驱动直接转发HI91度值；协议文档误写弧度 |
| 自瞄语义 | 绝对目标角、角速度等 | yaw/pitch及剩余角度误差 |
| 战术/裁判状态 | 期望丰富状态和动作字段 | 当前32字节接口不携带这些字段 |
| CRC检查配置 | require_crc_check=false | Decode强制CRC |

对应源码：protocol.hpp、hw_bridge_node.cpp、crc16.hpp和config/params.yaml。
对齐前虽然CRC算法一致，两端报文不能直接互通；当前适配已采用40/32字节布局。

## 原对齐建议（现已按此实现，控制语义限制仍保留）

先实现独立的40/32字节协议适配层与收包验证，随后再接入ROS话题。
导航速度与自瞄目标应在一个40字节帧中合成，不能继续分别发送SP和NAVI帧。
绝对目标到yaw_diff/pitch_diff的转换要核对参考系、单位与机构正方向，
并明确下位机多次应用旧误差的行为，不能直接套用此前XUC V1“一次纠偏”的发送策略。
对32字节中不存在的模式、弹速、裁判、血量和动作执行确认信息，应显式标记缺失，
不能用占位常量假装收到真实反馈。
现已可通过合法上行回包中的rx_count增长，验证双向基础链路。
若要确认某一条具体指令，还需双方定义下行序号及回显，不能仅凭累计计数作对应确认。
此前communication_minimal/upper.py按28字节回包解析，不能直接读取本次32字节上行；
需要同步修改接收帧长、CRC覆盖长度和新增字段解析。

## 本次更新核对

新增字段、CRC位置和32字节sizeof断言已对照git diff及当前源码核实。
队友仓库的协议文档第九节标题仍写“待实现”，但两个计数字段已经实际写入Encode发送流程。
本次同时包含yaw/pitch限位等其他代码修改，非只有通信修改。
仓库使用git pull --ff-only更新，本地未增加下位机源码改动。
