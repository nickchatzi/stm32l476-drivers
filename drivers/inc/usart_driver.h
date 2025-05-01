#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_

#include "stm32l476.h"
#include "rcc_driver.h"

/****************STRUCTURE FOR USARTx PIN CONFIGURATION****************/

typedef struct
{
    uint8_t USART_Mode;
    uint32_t USART_Baud;
    uint8_t USART_NoOfStopBits;
    uint8_t USART_Oversampling;
    uint8_t USART_WordLength;
    uint8_t USART_ParityControl;
    uint8_t USART_HWFlowControl;

}USART_Config;

/*********************************************************************/

/********************STRUCTURE FOR USARTx PIN HANDLE********************/

typedef struct
{
    USART_RegDef  *pUSARTx;         //Holds the base address (x:1,2,3) peripheral
    USART_Config  USART_Config;
    uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}
USART_Handle;

/*********************************************************************/

/**************************USART MODE MACROS***************************/

#define USART_MODE_ONLY_TX                  0
#define USART_MODE_ONLY_RX                  1
#define USART_MODE_TXRX                     2

/*********************************************************************/

/***********************USART BAUD RATE MACROS************************/

#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*********************************************************************/

/**************************USART PARITY MACROS************************/

#define USART_PARITY_DISABLE                0
#define USART_PARITY_EN_EVEN                1
#define USART_PARITY_EN_ODD                 2


/*********************************************************************/

/**********************USART OVERSAMPLING MACROS**********************/

#define USART_OVER16                        0
#define USART_OVER8                         1


/*********************************************************************/

/************************USART WORD LENGTH MACROS**********************/

#define USART_WORDLEN_7BITS                 0
#define USART_WORDLEN_8BITS                 1
#define USART_WORDLEN_9BITS                 2

/*********************************************************************/

/********************USART NUMBER OF STOP BITS MACROS******************/

#define USART_STOPBITS_1                    0
#define USART_STOPBITS_0_5                  1
#define USART_STOPBITS_2                    2
#define USART_STOPBITS_1_5                  3

/*********************************************************************/

/***********************USART FLOW CONTROL MACROS*********************/

#define USART_HW_FLOW_CTRL_NONE    	        0
#define USART_HW_FLOW_CTRL_CTS    	        1
#define USART_HW_FLOW_CTRL_RTS    	        2
#define USART_HW_FLOW_CTRL_CTS_RTS	        3

/*********************************************************************/

/********************CR1 REGISTER BIT DEFINITIONS*********************/

#define UE                                  0     
#define UESM                                1
#define RE                                  2
#define TE                                  3
#define IDLEIE                              4
#define RXNEIE                              5
#define TCIE                                6
#define TXEIE                               7
#define PEIE                                8
#define PS                                  9
#define PCE                                 10
#define WAKE                                11
#define M0                                  12
#define MME                                 13
#define CMIE                                14
#define OVER8                               15
#define DEDT                                16
#define DEAT                                21
#define RTOIE                               26
#define EOBIE                               27
#define M1                                  28

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define ADDM7                               4     
#define LBDL                                5
#define LBDIE                               6
#define LBCL                                8
#define CPHA                                9
#define CPOL                                10
#define CLKEN                               11
#define STOP                                12

/*********************************************************************/

/********************CR3 REGISTER BIT DEFINITIONS*********************/

#define EIE                                 0     
#define IREN                                1
#define IRLP                                2
#define HDSEL                               3
#define NACK                                4
#define SCEN                                5
#define DMAR                                6
#define DMAT                                7
#define RTSE                                8
#define CTSE                                9
#define CTSIE                               10
#define ONEBIT                              11
#define OVRDIS                              12
#define DDRE                                13
#define DEM                                 14
#define DEP                                 15

/*********************************************************************/

/********************ISR REGISTER BIT DEFINITIONS*********************/

#define PE                                  0     
#define FE                                  1
#define NF                                  2
#define ORE                                 3
#define IDLE                                4
#define RXNE                                5
#define TC                                  6
#define TXE                                 7
#define LBDF                                8
#define CTSIF                               9
#define CTS                                 10
#define RTOF                                11
#define EOBF                                12
#define ABRE                                14
#define ABRF                                15
#define BUSY                                16
#define CMF                                 17
#define SBKF                                18
#define RWU                                 19
#define WUF                                 20
#define TEACK                               21
#define REACK                               22
#define TCBGT                               25

/*********************************************************************/

/********************ICR REGISTER BIT DEFINITIONS*********************/

#define PECF                                 0     
#define FECF                                 1
#define NCF                                  2
#define ORECF                                3
#define IDLECF                               4
#define TCCF                                 6
#define TCBGTCF                              7
#define LBDCF                                8
#define CTSCF                                9
#define RTOCF                                11
#define EOBCF                                12
#define CMCF                                 17
#define WUCF                                 20

/*********************************************************************/

/********************USART STATUS FLAGS DEFINITIONS*********************/

#define USART_FLAG_TXE                        ( 1 << TXE)   //status to check if TX buffer is empty or not
#define USART_FLAG_RXNE                       ( 1 << RXNE)   //status to check if RX buffer is empty or not
#define USART_FLAG_TC                         ( 1 << TC)   //status to check if transmittion of data has ended or not

#define USART_READY                           0
#define USART_BUSY_IN_RX                      1
#define USART_BUSY_IN_TX                      2

#define USART_EVENT_TX_CMPLT                  0
#define	USART_EVENT_RX_CMPLT                  1
#define	USART_EVENT_IDLE                      2
#define	USART_EVENT_CTS                       3
#define	USART_EVENT_PE                        4
#define	USART_ERR_FE     	                  5
#define	USART_ERR_NF    	                  6
#define	USART_ERR_ORE    	                  7
#define USART_ERR_PE                          8
#define USART_ERR_CMF                         9

/*********************************************************************/

//Peripheral Clock Setup
void USART_PeriClockControl(USART_RegDef *pUSARTx, uint8_t EnorDi);

//Enable or Disable Peripheral
void USART_PeripheralControl(USART_RegDef *pUSARTx, uint8_t EnorDi);

//Init and Deinit
void USART_Init(USART_Handle *pUSARTHandle);
void USART_DeInit(USART_RegDef *pUSARTx);

//Send and Receive Data
void USART_SendData(USART_Handle *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//Send and Receive Data using Interrupts
uint8_t USART_SendDataIT(USART_Handle *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//Get status from the transmission
uint8_t USART_GetFlagStatus(USART_RegDef *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef *pUSARTx, uint8_t StatusFlagName);

//IRQ Config and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle *pHandle);

//Application Callback
void USART_ApplicationEventCallback(USART_Handle *pUSARTHandle,uint8_t AppEv);

//Set the baud rate
static void USART_SetBaudRate(USART_RegDef *pUSARTx, uint32_t BaudRate);


#endif
