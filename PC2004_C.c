

#include "PC2004_C.h"


//---������ ������� ��� ������ � ��������, �� ���� "������� ������" EN---//
void PulseLCD()
{
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,RESET);
    osDelay(1);
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,SET);
    osDelay(1);
    HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,RESET);
    osDelay(1);
}

//---������� ����� � �������---//
void SendByte(char ByteToSend, int IsData)
{
   HAL_GPIO_WritePin(PIN_D7_PORT,PIN_D7,(ByteToSend & 0x80));
   HAL_GPIO_WritePin(PIN_D6_PORT,PIN_D6,(ByteToSend & 0x40));
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,(ByteToSend & 0x20));
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,(ByteToSend & 0x10));
   HAL_GPIO_WritePin(PIN_D3_PORT,PIN_D3,(ByteToSend & 0x08));
   HAL_GPIO_WritePin(PIN_D2_PORT,PIN_D2,(ByteToSend & 0x04));
   HAL_GPIO_WritePin(PIN_D1_PORT,PIN_D1,(ByteToSend & 0x02));
   HAL_GPIO_WritePin(PIN_D0_PORT,PIN_D0,(ByteToSend & 0x01));   
   HAL_GPIO_WritePin(PIN_RS_PORT,PIN_RS,IsData);
   PulseLCD();         
   
}
 
//---��������� ������� �������---//
void Cursor(char Row, char Col)
{
   char address;
   if (Row == 0)
   address = 0;
   else if (Row == 1)
   address = 0x40;
   else if (Row == 2)
   address = 0x14;
   else 
   address = 0x54;
   
   address |= Col;
   SendByte(0x80 | address, 0);
}
 
//---������� �������---//
void ClearLCDScreen()
{
    SendByte(0x01, 0);
    SendByte(0x02, 0);  
}
 
//---������������� �������---//
void InitializeLCD(void)
{
   HAL_GPIO_WritePin(PIN_RS_PORT,PIN_RS,0);
   HAL_GPIO_WritePin(PIN_EN_PORT,PIN_EN,0);
   HAL_GPIO_WritePin(PIN_D7_PORT,PIN_D7,0);
   HAL_GPIO_WritePin(PIN_D6_PORT,PIN_D6,0);
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,1);
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,1);   
   osDelay(10);
   PulseLCD();
   HAL_GPIO_WritePin(PIN_D5_PORT,PIN_D5,1);
   HAL_GPIO_WritePin(PIN_D4_PORT,PIN_D4,1);
   PulseLCD();
   SendByte(0x3C, 0);  
   SendByte(0x0E, 0);  
   SendByte(0x06, 0);   
}
 
//---������ ������---//
void PrintStr(char *Text)
{
    char *c;
    c = Text;
    while ((c != 0) && (*c != 0))
    {
        SendByte(*c, 1);
        c++;
    }
}