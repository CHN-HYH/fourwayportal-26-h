# 项目记忆

- 项目：MSPM0 四路循迹与四路电机串口控制工程。
- 工程目录：`C:\ti\mspm0_sdk_2_02_00_05\FourWayPortal_USART`。
- 工具链：TI MSPM0 SDK，工程包含 Keil 相关文件和 SysConfig 配置。
- 当前编码规则：源码、头文件、脚本和文档统一使用无 BOM 的 GB2312（Windows-936）编码；转换时先按 UTF-8 严格解码确认内容，再编码为 GB2312。
- 已完成：2026-08-05 修复 `BSP/app_motor_usart.c` 中串口接收帧边界、短帧长度校验、分割数组边界和字段复制安全检查；未改变其他业务逻辑。
- 当前未处理问题：
  - P1：UART 发送无限等待；ISR 与主循环共享接收缓冲区缺少同步；循迹分支顺序和 80 ms 阻塞延时；`sprintf` 无长度限制。
  - P2：`Deal_data_real()` 未接入主流程；`V_y` 未使用；PID `error_last` 未更新；SysTick 延时硬编码和回卷计数风险；`Motor_Usart_init()` 只有声明没有实现。
- 本阶段未执行 Keil/TI 编译、烧录和实车验证。
