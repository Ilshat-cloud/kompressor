
#include "stm32f1xx_hal.h"
#ifndef PC2004_C
#define PC2004_C

#define     PIN_RS            GPIO_PIN_12          // PB12
#define     PIN_RS_PORT       GPIOB               
#define     PIN_EN            GPIO_PIN_13          // PB13
#define     PIN_EN_PORT       GPIOB               
#define     PIN_D7            GPIO_PIN_8          // PB8
#define     PIN_D7_PORT       GPIOB               
#define     PIN_D6            GPIO_PIN_12          // PA12
#define     PIN_D6_PORT       GPIOA               
#define     PIN_D5            GPIO_PIN_11          // PA11
#define     PIN_D5_PORT       GPIOA               
#define     PIN_D4            GPIO_PIN_10          // PA10
#define     PIN_D4_PORT       GPIOA               
#define     PIN_D3            GPIO_PIN_9          // PA9
#define     PIN_D3_PORT       GPIOA               
#define     PIN_D2            GPIO_PIN_8          // PA8
#define     PIN_D2_PORT       GPIOA               
#define     PIN_D1            GPIO_PIN_15          // PB15
#define     PIN_D1_PORT       GPIOB               
#define     PIN_D0            GPIO_PIN_14          // PB14
#define     PIN_D0_PORT       GPIOB              


void PulseLCD(void);
void SendByte(char ByteToSend, int IsData);
void Cursor(char Row, char Col);
void ClearLCDScreen(void);
void InitializeLCD(void);
void PrintStr(char *Text);
#endif
