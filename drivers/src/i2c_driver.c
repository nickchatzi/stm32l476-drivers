#include "i2c_driver.h"

static uint32_t timingSettings(I2C_Handle *pI2CHandle);

/*******************************I2C ENABLE**************************************
 
 * @fn          -I2C_PeripheralControl
 * 
 * @brief       -This function will enable the functionality of I2C.
 * 
 * @param[in]   -Base address of the I2C port
 * @param[in]   -Enable or Disable I2C
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_PeripheralControl(I2C_RegDef *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) 
    {
        pI2Cx->CR1 |= (1 << PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << PE);
    }
}
/*************************************************************************/

/**************************Peripheral Clock Setup*********************************
 
 * @fn          -I2C_PeripheralControl
 * 
 * @brief       -This function enables or disables peripheral clock for the given I2C port
 * 
 * @param[in]   -base address of the I2C peripheral
 * @param[in]   -ENABLE or DISABLE macros
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void I2C_PeriClockControl(I2C_RegDef *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }                          
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DIS();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DIS();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DIS();
        }           
    }
}
/*************************************************************************/

//Init and Deinit
void I2C_Init(I2C_Handle *pI2CHandle)
{
    uint32_t tempreg = 0;

    //Set the frequency of I2C communication
    tempreg |= timingSettings(pI2CHandle);
    pI2CHandle->pI2Cx->TIMINGR = tempreg;

    //Program the device own address
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    pI2CHandle->pI2Cx->OAR1 = tempreg;
}
/*************************************************************************/

/*******************************Deinit**************************************
 
 * @fn          -I2C_DeInit
 * 
 * @brief       -This function de-initializes the I2C and resets all the registers of a peripheral.
 * 
 * @param[in]   -Base address of the I2C port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/


void I2C_DeInit(I2C_RegDef *pI2Cx)
{
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
    else if (pI2Cx == I2C4)
    {
        I2C4_REG_RESET();
    }
}

//Get status from the transmission
uint8_t I2C_GetFlagStatus(I2C_RegDef *pI2Cx , uint32_t FlagName)
{

}
/*************************************************************************/

/***************************IRQ Interrupt Config***********************************
 
 * @fn          -I2C_IRQInterruptConfig
 * 
 * @brief       -Configure the NVIC registers to either enable or disable a given interrupt.
 * 
 * @param[in]   -interrupt number that needs to be enabled or disabled.
 * @param[in]   -ENABLE or DISABLE interrupt macros.
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= ( 1 << IRQNumber );
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
        }       
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if (IRQNumber >= 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
        }       
    }       
}
/*************************************************************************/

/***************************IRQ Priority Config***********************************
 
 * @fn          -I2C_IRQPriorityConfig
 * 
 * @brief       -Configure the priority for the given interrupt.
 * 
 * @param[in]   -interrupt number that needs to be enabled or disabled.
 * @param[in]   -Set the priority
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;  
    uint8_t iprx_section = IRQNumber % 4;  
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);  

    //*(NVIC_PR_BASE_ADDR + (iprx * 4)) &= ~(0xFF << shift_amount); //Clear previous priority
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount ); 
}
/*************************************************************************/

static uint32_t timingSettings(I2C_Handle *pI2CHandle)
{
    uint32_t temp;

    if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_SM)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_10KHZ)
        {
            temp |= ( 0xC7 << SCLL );
            temp |= ( 0xC3 << SCLH );
            temp |= ( 0x02 << SDADEL );
            temp |= ( 0x04 << SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x03 << PRESC );

            pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else if(pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_100KHZ)
        {
            temp |= ( 0x13 << SCLL );
            temp |= ( 0x0F << SCLH );
            temp |= ( 0x02 << SDADEL );
            temp |= ( 0x04 << SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x03 << PRESC );
        }
        else
        {
            fprintf(stderr, "Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }
    else if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_FM)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_200KHZ)
        {
            temp |= ( 0x3D << SCLL );
            temp |= ( 0x0B << SCLH );
            temp |= ( 0x00 << SDADEL );
            temp |= ( 0x01 << SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x00 << PRESC );

            pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else if(pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_400KHZ)
        {
            temp |= ( 0x09 << SCLL );
            temp |= ( 0x03 << SCLH );
            temp |= ( 0x02 << SDADEL );
            temp |= ( 0x03 << SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x01 << PRESC );
        }
        else
        {
            fprintf(stderr, "Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }
    else if(pI2CHandle->I2C_Config.I2C_Mode == I2C_SCL_SPEED_FMP)
    {
        if (pI2CHandle->I2C_Config.I2C_Freq == I2C_BUS_1000KHZ)
        {
            temp |= ( 0x04 << SCLL );
            temp |= ( 0x02 << SCLH );
            temp |= ( 0x00 << SDADEL );
            temp |= ( 0x00 << SCLDEL );
            //temp |= ( 0x00 << RES );
            temp |= ( 0x00 << PRESC );

            pI2CHandle->pI2Cx->TIMINGR = temp;
        }
        else
        {
            fprintf(stderr, "Error: Wrong frequency selection, in relation to the mode selection!\n");
        }
    }

    return temp;
}


void I2CApplicationEventCallback(I2C_Handle *pI2CHandle, uint8_t AppEv)
{
    if(pI2CHandle->I2C_Config.I2C_Freq == 0)
    {

    }
}
/*************************************************************************/


