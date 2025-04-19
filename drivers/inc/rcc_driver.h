#include "stm32l476.h"

typedef struct
{
    RCC_RegDef  *pRCC; 
    I2C_RegDef  *pI2Cx;

}RCC_Handle; 

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
void setHSIclock(void);

extern uint16_t AHB_PreScaler[8];
extern uint8_t APB1_PreScaler[4]; 
extern uint8_t APB2_PreScaler[4]; 