#ifndef _oled_h_
#define _oled_h_

#include <stdint.h>
#define OLED_ADDRESS 0x3c /* SSD1306 I2C 从机地址。 */

/* OLED 的 I2C 初始化接口，当前由 SysConfig 初始化替代。 */
void I2C_Configuration(void);
/* 向 OLED 写入一个控制字节和一个数据字节。 */
void I2C_WriteByte(uint8_t addr, uint8_t data);
/* 发送 OLED 控制指令。 */
void WriteCmd(unsigned char I2C_Command);
/* 发送 OLED 显存数据。 */
void WriteData(unsigned char I2C_Data);
/* 执行 SSD1306 初始化序列。 */
void OLED_Init(void);
/* 设置 OLED 页寻址模式下的显示坐标。 */
void OLED_SetPos(unsigned char x, unsigned char y);
/* 用同一字节填充全屏。 */
void OLED_Fill(unsigned char Fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
/* Steel 工程的 128x32 显存绘制接口。 */
void OLED_Refresh(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t mode);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr,
    uint8_t size, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *text,
    uint8_t size, uint8_t mode);
/* 按指定字号显示 ASCII 字符串。 */
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);

#endif
