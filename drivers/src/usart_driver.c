#include "usart_driver.h"

/************************** Peripheral Clock Setup *********************************
 
 * @fn          -USART_PeripheralControl
 * 
 * @brief       -This function enables or disables peripheral clock for the given USART port
 * 
 * @param[in]   -base address of the USART peripheral
 * @param[in]   -ENABLE or DISABLE macros
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void USART_PeriClockControl(USART_RegDef *pUSARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_EN();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_EN();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_EN();
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_EN();
        }                            
    }
    else
    {
        if (pUSARTx == USART1)
        {
            USART1_PCLK_DIS();
        }
        else if (pUSARTx == USART2)
        {
            USART2_PCLK_DIS();
        }
        else if (pUSARTx == USART3)
        {
            USART3_PCLK_DIS();
        }
        else if (pUSARTx == UART4)
        {
            UART4_PCLK_DIS();
        }
        else if (pUSARTx == UART5)
        {
            UART5_PCLK_DIS();
        }            
    }
}
/***********************************************************************************/

/******************************* USART ENABLE **************************************
 
 * @fn          -USARTPeripheralControl
 * 
 * @brief       -This function will enable the functionality of USART.
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -Enable or Disable USART
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
void USART_PeripheralControl(USART_RegDef *pUSARTx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) 
    {
        pUSARTx->CR1 |= (1 << UE);
    }
    else
    {
        pUSARTx->CR1 &= ~(1 << UE);
    }
}
/**************************************************************************/

/******************************* SET BAUD RATE **************************************
 
 * @fn          -USART_SetBaudRate
 * 
 * @brief       -This function will set the baud rate and will be called in the init function.
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -Pfefered baud rate
 * 
 * @return      -void
 * 
 * @note        -none
 
*/
 static void USART_SetBaudRate(USART_RegDef *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((2 * PCLKx) / BaudRate);
       usartdiv = round(usartdiv);

       uint8_t usartdiv_last_4_bits = usartdiv & 0xF;
       usartdiv_last_4_bits = usartdiv_last_4_bits >> 1;

       uint16_t usartdiv_rest_bits = usartdiv & 0xFFF0;

       tempreg |= (usartdiv_last_4_bits >> 0);
       tempreg |= (usartdiv_rest_bits >> 4);
  }
  else
  {
	   //over sampling by 16
	   usartdiv = (PCLKx / BaudRate);
       usartdiv = round(usartdiv);
       tempreg |= usartdiv;
  }
  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}

/******************************** Init **************************************
 
 * @fn          -USART_Init
 * 
 * @brief       -This function initializes the USART communication protocol
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void USART_Init(USART_Handle *pUSARTHandle)
{
	uint32_t tempreg=0;

    USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);

    /*Configure the mode (RX only, TX only or full duplex mode)*/
    if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
    {
        tempreg|= (1 << RE);
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
    {
        tempreg |= ( 1 << TE );
    }
    else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
    {
        tempreg |= ( ( 1 << RE) | ( 1 << TE) );
    }

    /*Configure the word length (7,8 or 9 data bits)*/
    if ( pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_7BITS)
    {
        tempreg|= (1 << M1);
        tempreg &= ~(1 << M0);
    }
    else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
    {
        tempreg &= ~(1 << M1);
        tempreg &= ~(1 << M0);
    }
    else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
    {
        tempreg &= ~(1 << M1);
        tempreg |= (1 << M0); 
    }    

    /*Configure the parity control (even, odd ore disable parity)*/
    if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
    {
        tempreg |= ( 1 << PCE);
        tempreg &= ~( 1 << PS);
    }
    else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
    {
        tempreg |= ( 1 << PCE);
        tempreg |= ( 1 << PS);
    }

    pUSARTHandle->pUSARTx->CR1 = tempreg;

    tempreg=0;

    /*Configure the STOP bit (0.5,1,1.5,2)*/
    tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << STOP;

    pUSARTHandle->pUSARTx->CR2 = tempreg;

    tempreg=0;
    
    /*Configure the flow control (CTS or RTS)*/
    if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
    {
        tempreg |= ( 1 << CTSE);
    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
    {
        tempreg |= ( 1 << RTSE);

    }
    else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
    {
        tempreg |= ( ( 1 << CTSE) | ( 1 << RTSE) );
    }

    pUSARTHandle->pUSARTx->CR3 = tempreg;

    USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}
/*****************************************************************************/

/******************************* Deinit **************************************
 
 * @fn          -USART_DeInit
 * 
 * @brief       -This function de-initializes the USART and resets all the registers of a peripheral.
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void USART_DeInit(USART_RegDef *pUSARTx)
{
    if (pUSARTx == USART1)
    {
        USART1_REG_RESET();
    }
    else if (pUSARTx == USART2)
    {
        USART2_REG_RESET();
    }
    else if (pUSARTx == USART3)
    {
        USART3_REG_RESET();
    }
    else if (pUSARTx == UART4)
    {
        UART4_REG_RESET();
    }
    else if (pUSARTx == UART5)
    {
        UART5_REG_RESET();
    }
}
/***********************************************************************************/

/********************************* Sent Data ****************************************
 
 * @fn          -USART_SendData
 * 
 * @brief       -This function sents data.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void USART_SendData(USART_Handle *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 7BIT, 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits 
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);
			
			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{ 
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
		{
			//This is 8bit data transfer 
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0xFF);		
			pTxBuffer++;
		}
        else
        {
            //This is 7bit data transfer 
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer  & (uint8_t)0x7F);
			pTxBuffer++;
        }
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}
/**************************************************************************************/

/********************************* Receive Data ****************************************
 
 * @fn          -USART_ReceiveData
 * 
 * @brief       -This function receives data.
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -void
 * 
 * @note        -none
 
*/

void USART_ReceiveData(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   for(uint32_t i = 0 ; i < Len; i++)
   {
       while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

       //Check the USART_WordLength to decide whether we are going to receive 9bit, 8 bit or 7bit of data in a frame
       if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
       {

           if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
           {
               *((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);
               pRxBuffer++;
               pRxBuffer++;
           }
           else
           {
                *pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
                pRxBuffer++;
           }
       }
       else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
       {
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
            }

            else
            {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);
            } 
            pRxBuffer++;
       }
       else 
       {
            if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
            {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);
            }   
            else
            {
                *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x3F);
            } 
            pRxBuffer++;
       }
   }
}
/***********************************************************************************/

/************************* Sent Data using Interrupt ********************************
 
 * @fn          -I2C_MasterSendDataIT
 * 
 * @brief       -
 * 
 * @param[in]   -Base address of the I2C peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -State of TXE bit
 * 
 * @note        -Non-blocking Call
 
*/

uint8_t USART_SendDataIT(USART_Handle *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->pUSARTx = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Enable interrupt for TXE
        pUSARTHandle->pUSARTx->CR1 |= ( 1 << TXEIE);
		
		//Enable interrupt for TC 	
        pUSARTHandle->pUSARTx->CR1 |= ( 1 << TCIE);
	}
	return txstate;
}
/***********************************************************************************/

/************************ Receive Data using Interrupt ******************************
 
 * @fn          -USART_ReceiveDataIT
 * 
 * @brief       -
 * 
 * @param[in]   -Base address of the SPI peripheral
 * @param[in]   -Pointer to the data
 * @param[in]   -Number of bytes of the packet
 * 
 * @return      -State of RXE bit
 * 
 * @note        -None
 
*/

uint8_t USART_ReceiveDataIT(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

        (void)pUSARTHandle->pUSARTx->RDR;

		//Enable interrupt for RXE
        pUSARTHandle->pUSARTx->CR1 |= ( 1 << RXNEIE);
    }
    return rxstate;
}
/**********************************************************************************************/

/************************************ Get Flag Status *****************************************
 
 * @fn          -USART_GetFlagStatus
 * 
 * @brief       -This function will return 1 if USART is busy (on-going transmission) and 0 if not.
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -Choose a flag from the USART_driver.h (STATUS FLAGS DEFINITIONS section)
 * @param[in]   -
 * 
 * @return      -1 or 0
 * 
 * @note        -None.
 
*/

uint8_t USART_GetFlagStatus(USART_RegDef *pUSARTx, uint8_t StatusFlagName)
{
	if(pUSARTx->ISR & StatusFlagName)
	{
		return FLAG_SET;
	}
    
	return FLAG_RESET;
}
/****************************************************************************************/

/************************************ Clear Flag *****************************************
 
 * @fn          -USART_ClearFlag
 * 
 * @brief       -This function will clear the flag.
 * 
 * @param[in]   -Base address of the USART port
 * @param[in]   -Choose a flag from the USART_driver.h (STATUS FLAGS DEFINITIONS section)
 * @param[in]   -
 * 
 * @return      -void
 * 
 * @note        -None.
 
*/
void USART_ClearFlag(USART_RegDef *pUSARTx, uint8_t StatusFlagName)
{
    pUSARTx->ISR &= ~( StatusFlagName);
}
/***********************************************************************************/

/*************************** IRQ Interrupt Config ***********************************
 
 * @fn          -USART_IRQInterruptConfig
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

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
/**********************************************************************************/

/*************************** IRQ Priority Config ***********************************
 
 * @fn          -USART_IRQPriorityConfig
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;  
    uint8_t iprx_section = IRQNumber % 4;  
    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);  

    //*(NVIC_PR_BASE_ADDR + (iprx * 4)) &= ~(0xFF << shift_amount); //Clear previous priority
    *(NVIC_PR_BASE_ADDR + iprx ) |= (IRQPriority << shift_amount ); 
}
/**************************************************************************/

void USART_IRQHandling(USART_Handle *pHandle)
{

}
/**************************************************************************/

void USART_ApplicationEventCallback(USART_Handle *pUSARTHandle,uint8_t AppEv)
{

}
/********************************END***************************************/
