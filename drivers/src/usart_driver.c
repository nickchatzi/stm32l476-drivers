#include "usart_driver.h"
#include "rcc_driver.h"

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

void USART_SetBaudRate(USART_RegDef *pUSARTx, uint32_t BaudRate)
{
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv = 0;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

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
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

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

    setHSIclock();

    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

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

    /*Configure the oversampling (OVER8 / OVER16)*/
    if (pUSARTHandle->USART_Config.USART_Oversampling == USART_OVER8)
    {
        tempreg |= ( 1 << OVER8);
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
 
 * @fn          -USART_SendDataIT
 * 
 * @brief       -
 * 
 * @param[in]   -Base address of the USART peripheral
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
		pUSARTHandle->TxLen = Len;
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
 * @param[in]   -Base address of the USART peripheral
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

/************************ IRQ Handling Config *******************************
 *
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -Configure the reason an interrupt occured and handle the appropriate interrupt.
 *
 * @param[in]         -Base address of the USART port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_IRQHandling(USART_Handle *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;

/*************************Check if Transmission is complete ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if (pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->ISR &= ~( 1 << TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else if ((pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS))
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
                else
                {
                    //This is 7bit data transfer
					pUSARTHandle->pUSARTx->TDR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0x7F);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
                }
			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << RXNEIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->RDR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
                    else
                    {
                        *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);
                        pUSARTHandle->pRxBuffer++;
                        pUSARTHandle->RxLen-=1;
                    }
				}
				else
				{
					//check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
					{
						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0xFF);
					}
					else if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
					{
						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x7F);
					}
                    else
                    {
                        //read only 6 bits , hence mask the DR with 0X7F
                        *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->RDR  & (uint8_t)0x3F);
                    }
					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}
			}
			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

/*************************Check for CTS flag ********************************************/

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << CTSIF);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << CTSIE);

	if(temp1 && temp2 && temp3)
	{
		pUSARTHandle->pUSARTx->ICR |= ( 1 << CTSCF);
        pUSARTHandle->pUSARTx->CR3 &=  ~( 1 << CTSIE);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & ( 1 << IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << IDLEIE);

	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag.
		pUSARTHandle->pUSARTx->ICR |= ( 1 << IDLECF);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & (1 << ORE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << RXNEIE);

	if(temp1  && temp2 )
	{
        pUSARTHandle->pUSARTx->ICR |= ( 1 << ORECF);

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}

/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << EIE) ;

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->ISR;

		if(temp1 & ( 1 << FE) )
		{
            pUSARTHandle->pUSARTx->ICR |= ( 1 << FECF);

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & ( 1 << NF) )
		{
            pUSARTHandle->pUSARTx->ICR |= ( 1 << NCF);

			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NF);
		}

		if(temp1 & ( 1 << ORE) )
		{
			pUSARTHandle->pUSARTx->ICR |= ( 1 << ORECF);

            USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}

    /*************************Check for parity error******************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & (1 << PE);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 <<PEIE);

	if(temp1  && temp2 )
	{
        pUSARTHandle->pUSARTx->ICR |= ( 1 << PECF);

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_PE);
	}

    /*************************Check for character match******************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->ISR & (1 << CMF);

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 <<CMIE);

	if(temp1  && temp2 )
	{
        pUSARTHandle->pUSARTx->ICR |= ( 1 << CMCF);

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_CMF);
	}
}
/**************************************************************************/

void USART_ApplicationEventCallback(USART_Handle *pUSARTHandle,uint8_t AppEv)
{

}
/********************************END***************************************/
