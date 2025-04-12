#include "stm32l476.h"

typedef struct
{
    RCC_RegDef  *pRCC; 

}RCC_Handle; 

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
void setHSIclock(RCC_RegDef *pRCC);

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};