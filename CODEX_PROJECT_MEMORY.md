# 项目记忆

- 项目：MSPM0 四路循迹与四路电机串口控制工程。
- 工程目录：`C:\ti\mspm0_sdk_2_02_00_05\FourWayPortal_USART`。
- 工具链：TI MSPM0 SDK，工程包含 Keil 相关文件和 SysConfig 配置。
- 当前编码规则：源码、头文件、脚本和文档统一使用无 BOM 的 UTF-8 编码；不要再转回 GB2312。
- 已完成：2026-08-05 修复 `BSP/app_motor_usart.c` 中串口接收帧边界、短帧长度校验、分割数组边界和字段复制安全检查；未改变其他业务逻辑。
- 当前未处理问题：
  - P1：UART 发送无限等待；ISR 与主循环共享接收缓冲区缺少同步；循迹分支顺序和 80 ms 阻塞延时；`sprintf` 无长度限制。
  - P2：`Deal_data_real()` 未接入主流程；`V_y` 未使用；PID `error_last` 未更新；SysTick 延时硬编码和回卷计数风险；`Motor_Usart_init()` 只有声明没有实现。
- UART3 摄像头输入：UART3 使用 PA13(RX)/PA14(TX)，MFCLK 4 MHz，115200 8N1，启用 RX 中断；外部发送端 TX 应连接 PA13，双方必须共地。`BSP/bsp_camera_usart.c/.h` 已实现 128 字节环形缓冲、AA 55 帧状态机、前 7 字节累加和低 8 位校验和 `vision` 状态；UART3 ISR 仅收字节，主循环调用 `Camera_Vision_Process()` 解析。TIMG12 提供独立的 1 ms 计时，未影响原 SysTick 延时。
- 本阶段未执行 Keil/TI 编译、烧录和实车验证。
- 已完成 UART3 摄像头链路验证：摄像头经 PA13 向 MSPM0 UART3 发送数据，MSP 曾通过 UART0 的 PA10 原样输出供串口助手确认；测试回发逻辑已移除。当前 UART3 ISR 仅将原始字节写入接收缓冲，主循环负责 8 字节协议解析。
- UniFlash 串口 BSL 的镜像 8 字节对齐：修改前 `Keil/empty.map` 的 `LR_IROM1 Size` 为 `0x9BC`（余 4）；已在 `empty.c` 函数外加入 8 字节 `__attribute__((used)) const uint32_t g_uniflash_padding[2]`。本次舵机加入后旧镜像实际为 `0x1014`，重新编译后预期为 `0x1018`；必须以新生成的 `OBJ/empty.hex` 和 `.map` 为准验证，不能手改旧 hex。
- 摄像头协议处理：仅累加和校验正确帧更新 `last_rx_ms`；仅 `FLAGS & 1` 且 `x < 320` 的新序号帧更新 x、`last_valid_ms` 和 valid。无效标志、越界、重复序号、跳号、校验和错误和环形缓冲溢出均单独记录，控制可通过 `Camera_Vision_IsUsable()` 使用 250 ms 失效阈值。舵机标定、像素到 cm 换算和 PID 尚未接入。
- UART0 解析联调输出：`BSP/bsp_camera_usart.c` 中的 `CAMERA_VISION_DEBUG_ENABLE` 当前为 `1U`，`empty.c` 已恢复 `USART_Init()`、`Camera_Vision_Init()` 和主循环 `Camera_Vision_Process()`。每处理一帧会输出有效帧（seq/x/width）、无目标帧、校验和错误、坐标越界或重复序号；验证完成后设为 `0U`，避免逐帧串口打印占用主循环时间。
- 2026-08-05 摄像头到 MSPM0 实测：UART0 调试日志累计 676 帧，SEQ 全程连续且包含 255->0 正常回绕；未出现校验和错误、坐标越界或重复序号。有效帧 x 范围为 57~289，width 范围为 8~102；带时间戳的 630 帧平均间隔 77.2 ms、最大 124 ms。通信和协议解析已验证正常；`valid=0` 连续出现时应由后续控制进入安全状态，width 很小的有效候选帧暂只记录、待实车标定质量阈值。
- 按键与 LED 联调配置（2026-08-06，已实测）：K1=PA2、K2=PB19、K3=PB20、K4=PA23，均采用低电平按下、无内部上下拉；LED1=PB2、LED2=PB3，均为高电平点亮。`BSP/key.c/.h` 提供 `Key_GetEvent()`，经过 20 ms 二次确认消抖，每次按下仅返回一次 `KEY_EVENT_K1` 至 `KEY_EVENT_K4`，全松开后可再次触发。LED 映射测试逻辑已移除；电机、循迹和摄像头业务仍保持注释。
- 舵机单路测试配置（2026-08-06，待实机验证）：选用扩展板 S2=PB9，复用为 TIMA0_CCP1；SysConfig 固定仅启用 CCP1，定时器时钟 1 MHz、周期 20000 tick（50 Hz），未占用 S1=PB8、PA0 或互补输出引脚。新增 BSP/servo.c/.h，提供 Servo_Init() 与 Servo_SetAngle()，按 500 us~2500 us 映射 0°~180°；empty.c 舵机测试逻辑已移除，当前主循环用于摄像头 UART0 解析联调。Keil 工程已加入 BSP/servo.c。舵机须使用独立稳定的 5 V 供电，并与 MSP 共地。
- Git 远程连接（2026-08-06）：`origin` 使用 SSH 地址 `git@github.com:CHN-HYH/fourwayportal-26-h.git`。此前仓库级 `http.proxy`/`https.proxy` 指向未监听的 `127.0.0.1:7892`，导致 HTTPS 推送失败；现已删除这两条本地代理配置，并通过 `git ls-remote origin HEAD` 验证 SSH 访问正常。
- 视觉舵机 P 控制测试（2026-08-06）：新增 `BSP/vision_servo_test.c/.h`，每 20 ms 使用 `vision.x` 相对 `x_center=160` 的像素误差计算舵机角度，初始 `Kp=0.20`、舵机中位 90°、方向系数 1.0；视觉无效或超过 250 ms 未更新时回到 90°。方向、中心点和 Kp 均可在 `vision_servo_test.c` 顶部调整，尚未实机验证。`empty.c` 仅保留初始化和接口调用。
- OLED 适配（2026-08-06）：在 `empty.syscfg` 中加入 I2C0，使用 PA28(SDA)/PA31(SCL)、400 kHz，已通过 SDK SysConfig CLI 重新生成 `ti_msp_dl_config.c/.h`。迁移 `BSP/oled.c/.h` 与 `BSP/codetab.h`，新增 `BSP/oled_timer_test.c/.h`；测试调用已有 K1（PA2）接口，K1 按下后从 0 秒开始在 OLED 中央计时，再次按下重新计时。
- OLED 计时接口保留（2026-08-06）：移除 K1 按键测试和 `empty.c` 中的 OLED 测试调用；保留 `BSP/oled.c/.h`、字库及计时模块，公开 `Oled_Timer_Init()`、`Oled_Timer_Start()`、`Oled_Timer_Update()`，显示固定为居中的大号 `00:00`。
- I2C OLED 计时测试恢复（2026-08-06）：当前使用 PA28(SDA)/PA31(SCL) 的 I2C0 OLED；`empty.c` 初始化 `Oled_Timer_Init()`，K1（PA2）按下调用 `Oled_Timer_Start()`，主循环调用 `Oled_Timer_Update()`，显示居中的大号 `00:00`。SPI 屏幕方案暂不迁移。

- 代码可读性整理（2026-08-11）：手写源码、头文件、SysConfig、脚本和文档已统一为无 BOM 的 UTF-8。内部变量优先使用简短名称（如 `min`、`max`、`cnt`、`idx`、`s_rx`），在声明处以中文注释说明用途、单位或数据来源；不为 `i` 等上下文充分的循环变量机械添加注释。自动生成文件与对外函数符号保持不变。本次未执行 Keil 编译、烧录和实车验证。

- 2026-08-11 舵机 0 度固定测试：`empty.c` 改为调用 `Servo_Init()` 后显式执行 `Servo_SetAngle(0U)`，主循环暂不调用视觉舵机更新，适用于验证 PB9/TIMA0_CCP1 的 0 度 PWM 输出；本次未执行 Keil 编译、烧录和实机验证。

- 2026-08-11 钢珠视觉 PD 控制融合：`BSP/vision_servo_test.c` 改为仅在新有效视觉帧到达时更新，按位置误差和滤波速度计算 `Kp * error - Kd * velocity` 的角度增量；视觉无效时回到平衡基准角。当前初始参数为目标 +5 cm、基准 15 度、限幅 0~30 度、方向 -1、像素换算 0.075 cm/px、Kp=1.00、Kd=0.20；像素换算基于水管总长 25 cm、钢球直径 1 cm，因此钢球中心行程 -12~+12 cm 对应 ROI 320 px。其余参数均须实车标定。`empty.c` 已恢复调用视觉舵机初始化与更新；`CAMERA_VISION_DEBUG_ENABLE` 设为 0，避免逐帧串口输出增加控制滞后。未执行 Keil 编译、烧录和实车验证。

- 代码命名准则（2026-08-11）：控制模块的可调宏使用简短模块前缀（例如 `SV_`），模块静态状态使用 `s_`，局部变量使用简短且含义明确的名称；宏、静态变量和关键局部变量均应附简短中文注释，避免冗长英文变量名。

- 钢珠视觉定点控制（2026-08-12，待实车验证）：`BSP/vision_servo_test.c/.h` 当前使用带保持角补偿的 PD 闭环让钢珠稳定在固定的 `+12 cm`，不再到位后切换 `-12 cm`。实测摆杆水平为 `30°`，且用户日志确认钢珠稳定在 +12 cm 时积分补偿约为 `+12°`、舵机角约为 `42°`；因此以 `SV_ANG_HOLD=42°` 作为定点保持角，移除会导致等待时间长和过冲的积分项。控制公式为 `angle=42 + Kp*error - Kd*velocity`，并把定点运行角度上限限制为 `45°`，避免远距离时打到 `50°` 导致高速、识别丢失。当前参数：范围 `10°~50°`、`Kp=1.0`、`Kd=0.35`、每新视觉帧最大变角 `2°`。视觉失效或通信超时时仍回 `30°` 水平安全角。未执行 Keil 编译、烧录或实车验证。

- 舵机机械行程实测（2026-08-12）：用户确认舵机输入 `10°` 时钢珠向负方向运动，`50°` 时钢珠向正方向运动，`30°` 为摆杆水平。视觉定点控制已据此更新为 `SV_ANG0=30°`、`SV_ANG_MIN=10°`、`SV_ANG_MAX=50°`、`SV_DIR=1.0`。此前 `SV_ANG0=25°` 会在接近 +12 cm 时把舵机压到实际水平角以下，导致钢珠向负方向回滚，是中途无法稳定的主要原因。

- 视觉位置实测标定（2026-08-12，待继续验证）：用户测得旧公式 `pos=(x-160)*0.075` 时，实际 `12 cm` 显示 `10.35 cm`，实际 `5.6 cm` 显示 `4.5 cm`，实际 `6.1 cm` 显示 `4.95 cm`。按这些数据作线性标定，更新 `SV_X0=152.0`、`SV_CM_PX=0.082`；对应 `pos_new=(pos_old+0.6)*1.0933`，可使上述三个点约对应 11.97、5.58、6.07 cm。未编译、烧录或实车验证。
