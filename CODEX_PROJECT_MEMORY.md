# 项目记忆

- 项目：MSPM0 四路循迹与四路电机串口控制工程。
- 工程目录：`C:\ti\mspm0_sdk_2_02_00_05\FourWayPortal_USART`。
- 工具链：TI MSPM0 SDK，工程包含 Keil 相关文件和 SysConfig 配置。
- 当前编码规则：源码、头文件、脚本和文档统一使用无 BOM 的 GB2312（Windows-936）编码；转换时先按 UTF-8 严格解码确认内容，再编码为 GB2312。
- 已完成：2026-08-05 修复 `BSP/app_motor_usart.c` 中串口接收帧边界、短帧长度校验、分割数组边界和字段复制安全检查；未改变其他业务逻辑。
- 当前未处理问题：
  - P1：UART 发送无限等待；ISR 与主循环共享接收缓冲区缺少同步；循迹分支顺序和 80 ms 阻塞延时；`sprintf` 无长度限制。
  - P2：`Deal_data_real()` 未接入主流程；`V_y` 未使用；PID `error_last` 未更新；SysTick 延时硬编码和回卷计数风险；`Motor_Usart_init()` 只有声明没有实现。
- UART3 摄像头接收配置：UART3 使用 PA13(RX)/PA14(TX)，MFCLK 4 MHz，115200 8N1，启用 RX 中断；外部发送端 TX 应连接 PA13，双方必须共地。`BSP/bsp_camera_usart.c/.h` 提供 256 字节原始接收缓冲、有效长度、接收标志和清空函数，已加入 Keil 工程；尚未实现摄像头协议解析。
- 本阶段未执行 Keil/TI 编译、烧录和实车验证。
- 已完成 UART3 摄像头链路验证：摄像头经 PA13 向 MSPM0 UART3 发送数据，MSP 曾通过 UART0 的 PA10 原样输出供串口助手确认；测试回发逻辑已移除。当前 UART3 ISR 仅将原始字节写入接收缓冲，待实现 8 字节协议解析。
- UniFlash 串口 BSL 的镜像 8 字节对齐：修改前 `Keil/empty.map` 的 `LR_IROM1 Size` 为 `0x9BC`（余 4）；已在 `empty.c` 函数外加入 `__attribute__((used)) const uint32_t g_uniflash_padding = 0xFFFFFFFFU`。重新编译后预期为 `0x9C0`；必须以新生成的 `OBJ/empty.hex` 和 `.map` 为准验证，不能手改旧 hex。
