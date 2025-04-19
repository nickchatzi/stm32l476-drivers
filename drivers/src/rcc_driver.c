#include "rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};

uint8_t RCC_GetPLLOutputClock()
{
	return 0;
}

uint8_t RCC_GetMSIOutputClock()
{
	return 0;
}

/*CALCULATE THE PCLK1*/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> 2) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = 16000000;    
	}
	else if (clksrc == 2)
	{
		SystemClk = 8000000;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> 8) & 0x7;

	if (temp < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1_PreScaler[temp-4];
	}
	pclk1 = (SystemClk/ahbp) / apb1p;

	return pclk1;
}

/*CALCULATE THE PCLK2*/
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = (RCC->CFGR >> 2) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = 16000000;    
	}
	else if (clksrc == 2)
	{
		SystemClk = 8000000;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> 11) & 0x7;

	if (temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB2_PreScaler[temp-4];
	}
	pclk1 = (SystemClk/ahbp) / apb2p;

	return pclk1;
}

void setHSIclock()
{
// 1. Enable HSI clock
RCC->CR |= (1 << 8);  // Set HSION bit

// 2. Wait for HSI to be ready
while (!(RCC->CR & (1 << 10)));  // Wait for HSIRDY to be set

// 3. Select HSI as system clock
RCC->CFGR |= (1 << 0);  // Set SW to 01 (HSI selected)

// 4. Wait until the switch is complete
while ((RCC->CFGR & (3 << 2)) != (1 << 2));  // Wait until SWS = 01 (HSI is used)

}