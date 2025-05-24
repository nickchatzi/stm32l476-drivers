#include "rcc_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = {2,4,8,16};
uint8_t APB2_PreScaler[4] = {2,4,8,16};
uint16_t MSI_Range[12] = {100,200,400,800,1000,2000,4000,8000,16000,24000,32000,48000};

typedef struct
{
	uint8_t pllp;
	uint8_t pllq;
	uint8_t pllr;
}PLLDividers;

static void assert_failed(uint8_t *file, uint32_t line);

static uint8_t RCC_GetPLLOutputClock()
{
	return 0;
}

static uint16_t RCC_GetMSIOutputClock()
{
	uint8_t temp;
	uint16_t msiclk;

	temp = (RCC->CR >> MSIRANGE) & 0xF;

	msiclk = MSI_Range[temp * 1000];

	return msiclk;
}

/*CALCULATE THE PCLK2*/
uint32_t RCC_GetCLKValue(void)
{
	uint32_t clksrc;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		return RCC_GetMSIOutputClock();
	}
	else if (clksrc == 1)
	{
		return HSI;    
	}
	else if (clksrc == 2)
	{
		return HSE;   
	}
	else
    {
        return 0;
    }
}

/*CALCULATE THE PCLK1*/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1p;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = HSI;    
	}
	else if (clksrc == 2)
	{
		SystemClk = HSE;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> HPRE) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> PPRE1) & 0x7;

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
	uint32_t pclk2, SystemClk;

	uint8_t clksrc, temp, ahbp, apb2p;

	clksrc = (RCC->CFGR >> SWS) & 0X3;

	if (clksrc == 0)
	{
		SystemClk = RCC_GetMSIOutputClock();   
	}
	else if (clksrc == 1)
	{
		SystemClk = HSI;    
	}
	else if (clksrc == 2)
	{
		SystemClk = HSE;   
	}
    else if (clksrc == 3)
	{
		SystemClk = RCC_GetPLLOutputClock();   
	} 

	temp = (RCC->CFGR >> HPRE) & 0xF;

	if (temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	temp = (RCC->CFGR >> PPRE2) & 0x7;

	if (temp < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB2_PreScaler[temp-4];
	}
	pclk2 = (SystemClk/ahbp) / apb2p;

	return pclk2;
}

/******************************* SET HSI CLOCK **************************************
 
 * @fn          -setHSIclock
 * 
 * @brief       -This function will set HSI as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setHSIclock(RCC_Handle *pRCCHandle)
{
	uint32_t temp = 0;

	// Clear MSI bit
	pRCCHandle->pRCC->CR &= ~(1 << MSION);

	// Enable HSI clock
	pRCCHandle->pRCC->CR |= (1 << HSION);

	// Wait for HSI to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << HSIRDY))); 

	// Select HSI as system clock 
	pRCCHandle->pRCC->CFGR  |= (1 << SW);  

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (1 << SWS));  

	temp = pRCCHandle->pRCC->CFGR;

	//Configure the AHB prescaler
	temp &= ~(0xF << HPRE);
	temp |= (pRCCHandle->RCC_Config.RCC_AHB_Presc << HPRE);

	//Configure the APB1(Low Speed) prescaler
	temp &= ~(0x7 << PPRE1);
	temp |= (pRCCHandle->RCC_Config.RCC_APB1 << PPRE1);

	//Configure the APB2(High Speed) prescaler
	temp &= ~(0x7 << PPRE2);
	temp |= (pRCCHandle->RCC_Config.RCC_APB2 << PPRE2);
	
	pRCCHandle->pRCC->CFGR = temp; 
	
}

/******************************* SET HSE CLOCK **************************************
 
 * @fn          -setHSEclock
 * 
 * @brief       -This function will set HSE as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setHSEclock(RCC_Handle *pRCCHandle)
{
	uint32_t temp = 0;

	// Enable HSE clock
	pRCCHandle->pRCC->CR |= (1 << HSEON);  

	// Wait for HSE to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << HSERDY))); 

	temp = pRCCHandle->pRCC->CFGR;
	// Select HSE as system clock
	temp |= (0x2 << SW);  

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (0x2 << SWS));  

	//Configure the AHB prescaler
    temp &= ~(0xF << HPRE);
    temp |= (pRCCHandle->RCC_Config.RCC_AHB_Presc << HPRE);

	//Configure the APB1(Low Speed) prescaler
    temp &= ~(0x7 << PPRE1);
    temp |= (pRCCHandle->RCC_Config.RCC_APB1 << PPRE1);

	//Configure the APB2(High Speed) prescaler
    temp &= ~(0x7 << PPRE2);
    temp |= (pRCCHandle->RCC_Config.RCC_APB2 << PPRE2);

	pRCCHandle->pRCC->CFGR = temp;

}

/******************************* SET MSI CLOCK **************************************
 
 * @fn          -setMSIclock
 * 
 * @brief       -This function will set MSI as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setMSIclock(RCC_Handle *pRCCHandle)
{
	// Configure MSI range clock frequency after standby mode
    pRCCHandle->pRCC->CSR &= ~(0xF << MSISRANGE);
    pRCCHandle->pRCC->CSR |= (pRCCHandle->RCC_Config.RCC_MSIRangeStandby << MSISRANGE);

	pRCCHandle->pRCC->CR |= (1 << MSIRGSEL);

    // Configure MSI range clock frequency
    pRCCHandle->pRCC->CR &= ~(0xF << MSIRANGE);
    pRCCHandle->pRCC->CR |= (pRCCHandle->RCC_Config.RCC_MSIRange << MSIRANGE);
	
	// Enable MSI clock
	pRCCHandle->pRCC->CR |= (1 << MSION);  

	// Wait for MSI to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << MSIRDY)));  

	// Select MSI as system clock
	pRCCHandle->pRCC->CFGR &= ~(0x3 << SW); // Clear SW bits

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (0x0 << SWS));  
}

/******************************* SET PLL CLOCK **************************************
 
 * @fn          -setPLLclock
 * 
 * @brief       -This function will set PLL as the default clock source.
 * 
 * @param[in]   -Base address of the RCC register
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void setPLLclock(RCC_Handle *pRCCHandle)
{
	uint32_t temp = 0;

	PLLDividers pll_dividers;

	uint8_t pllm_value = (pRCCHandle->RCC_Config.RCC_PLLM) + 1; 
	uint8_t plln_value = pRCCHandle->RCC_Config.RCC_PLLN;
	uint8_t pllp_value = pll_dividers.pllp;	
	uint8_t pllq_value = pll_dividers.pllq;
	uint8_t pllr_value = pll_dividers.pllr;

	uint32_t SystemClk = RCC_GetCLKValue();
	uint32_t vco_input = SystemClk / pllm_value;
	uint32_t vco_output = vco_input * plln_value;

	uint32_t pllpOutputClk = vco_output / pllp_value;
	uint32_t pllqOutputClk = vco_output / pllq_value;
	uint32_t pllrOutputClk = vco_output / pllr_value;

	// Check validity
	assert_param(vco_input >= 4000000 && vco_input <= 16000000);
	assert_param(vco_output >= 64000000 && vco_output <= 344000000);
	assert_param (pllpOutputClk <=64000000 && pllqOutputClk <=64000000 && pllrOutputClk <=64000000);

	//Disable PLL clock
	pRCCHandle->pRCC->CR &= ~(1 << PLLON);

	while (!(pRCCHandle->pRCC->CR & (1 << PLLRDY)));  

	if (SystemClk == HSI)
	{
		temp |= (2 << PLLSRC);
	}
	else if (SystemClk == HSE)
	{
		temp |= (3 << PLLSRC);
	}
	//else any other value for SystemClk means MSI is being used as system clock
	else
	{
		temp |= (1 << PLLSRC);
	}

	//Configure the M divider (PLLM)
	temp &= ~(0x7 << PLLM);
	temp |= (pRCCHandle->RCC_Config.RCC_PLLM << PLLM);

	//Configure the N multiplier (PLLN)
	temp &= ~(0x7F << PLLN);
	temp |= (plln_value << PLLN);

	//Configure the P divider (PLLP)
	if (pRCCHandle->RCC_Config.RCC_PLLP < 2)
	{
		temp |= (1 << PLLPEN);
		temp &= ~(1 << PLLP);
		temp |= (pllp_value << PLLP);
	}

	//Configure the Q divider (PLLQ)
	if (pRCCHandle->RCC_Config.RCC_PLLQ < 4)
	{
		temp |= (1 << PLLQEN);
		temp &= ~(3 << PLLQ);
		temp |= (pllq_value << PLLQ);
	}

	//Configure the R divider (PLLR)
	if (pRCCHandle->RCC_Config.RCC_PLLR < 4)
	{
		temp |= (1 << PLLREN);
		temp &= ~(3 << PLLR);
		temp |= (pllr_value << PLLR);
	}

	pRCCHandle->pRCC->PLLCFGR = temp;

	// Enable PLL clock
	pRCCHandle->pRCC->CR |= (1 << PLLON);  

	// Wait for PLL to be ready
	while (!(pRCCHandle->pRCC->CR & (1 << PLLRDY))); 
	
	// Select PLL as system clock
	pRCCHandle->pRCC->CFGR |= (0x3 << SW); // Clear SW bits

	// Wait until the switch is complete
	while ((pRCCHandle->pRCC->CFGR & (3 << SWS)) != (0x3 << SWS)); 

}

/*
Help function to return the actual divider values for P, Q and R
*/
static PLLDividers returnVariables(RCC_Handle *pRCCHandle)
{
	PLLDividers pll_dividers;

	switch (pRCCHandle->RCC_Config.RCC_PLLP)
	{
		case 0:
			pll_dividers.pllp = 7;
			break;
		case 1:
			pll_dividers.pllp = 17;
			break;
	}

	switch (pRCCHandle->RCC_Config.RCC_PLLQ)
	{
		case 0:
			pll_dividers.pllq = 2;
			break;
		case 1:
			pll_dividers.pllq = 4;
			break;
		case 2:
			pll_dividers.pllq = 6;
			break;
		case 3:
			pll_dividers.pllq = 8;
			break;
	}

	switch (pRCCHandle->RCC_Config.RCC_PLLR)
	{
		case 0:
			pll_dividers.pllr = 2;
			break;
		case 1:
			pll_dividers.pllr = 4;
			break;
		case 2:
			pll_dividers.pllr = 6;
			break;
		case 3:
			pll_dividers.pllr = 8;
			break;
	}
}		

static void assert_failed(uint8_t *file, uint32_t line)
{
    while(1);
}