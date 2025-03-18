
#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include "stm32l476.h"

/****************STRUCTURE FOR I2Cx PIN CONFIGURATION****************/

typedef struct
{
    uint8_t I2C_Freq;
    uint8_t I2C_Mode;
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
    uint8_t I2C_ACKControl;
    uint8_t I2C_FMDutyCycle;

}I2C_Config;

/*********************************************************************/

/********************STRUCTURE FOR I2Cx PIN HANDLE********************/

typedef struct
{
    I2C_RegDef  *pI2Cx;         //Holds the base address (x:1,2,3) peripheral
    I2C_Config  I2C_Config;

}I2C_Handle;

/*********************************************************************/

/**********************FREQ CONFIGURATION MACROS**********************/

#define I2C_BUS_10KHZ                       0
#define I2C_BUS_100KHZ                      1
#define I2C_BUS_200KHZ                      2
#define I2C_BUS_400KHZ                      3
#define I2C_BUS_1000KHZ                     4

/*********************************************************************/

/**********************SPEED CONFIFURATION MACROS**********************/

#define I2C_SCL_SPEED_SM                    0
#define I2C_SCL_SPEED_FM                    1
#define I2C_SCL_SPEED_FMP                   2

/*********************************************************************/

/**********************SPEED CONFIFURATION MACROS**********************

#define I2C_SCL_SPEED_SM                    100000
#define I2C_SCL_SPEED_FM2K                  200000
#define I2C_SCL_SPEED_FM4K                  400000

/*********************************************************************/

/******************ACK CONTROL CONFIGURATION MACROS*******************/

#define I2C_ACK_DISABLE                     0
#define I2C_ACK_ENABLE                      1               

/*********************************************************************/

/*******************DUTY CICLE CONFIGURATION MACROS*******************/

#define I2C_FM_DUTY_2                       ((1 << 28) | (4 << 20) | (2 << 16) | (16 << 8) | 8)              
#define I2C_FM_DUTY_16_9                    ((1 << 28) | (4 << 20) | (2 << 16) | (16 << 8) | 9)

/*********************************************************************/

/********************CR1 REGISTER BIT DEFINITIONS*********************/

#define PE                                  0     
#define TXIE                                1
#define RXIE                                2
#define ADDRIE                              3
#define NACKIE                              4
#define STOPIE                              5
#define TCIE                                6
#define ERRIE                               7
#define DNF                                 8
#define ANFOFF                              12
#define TXDMAEN                             14
#define RXDMAEN                             15
#define SCB                                 16
#define NOSTRETCH                           17

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define SADD                                0    
#define RD_WRN                              10
#define ADD10                               11
#define HEAD10R                             12
#define START                               13
#define STOP                                14
#define NACK                                15
#define NBYTES                              16
#define RELOAD                              24
#define AUTOEND                             25
#define PECBYTE                             26

/*********************************************************************/

/********************CR2 REGISTER BIT DEFINITIONS*********************/

#define SCLL                                0    
#define SCLH                                8
#define SDADEL                              16
#define SCLDEL                              20
#define RES                                 24
#define PRESC                               28

/*********************************************************************/

/********************ISR REGISTER BIT DEFINITIONS*********************/

#define TXE                                 0    
#define TXIS                                1
#define RXNE                                2
#define ADDR                                3
#define NACKF                               4
#define STOPF                               5
#define TC                                  6
#define TCR                                 7
#define BERR                                8
#define ARLO                                9
#define OVR                                 10
#define PECERR                              11
#define TIMEOUT                             12
#define ALERT                               13
#define BUSY                                15
#define DIR                                 16
#define ADDCODE                             17

/*********************************************************************/

/********************I2C STATUS FLAGS DEFINITIONS*********************/

#define I2C_TXE_FLAG                        ( 1 << TXE)   //status to check if TX buffer is empty or not
#define I2C_RXNE_FLAG                       ( 1 << RXNE)   //status to check if RX buffer is empty or not
#define I2C_ADDR_FLAG                       ( 1 << ADDR)
#define I2C_NACKF_FLAG                      ( 1 << NACKF)
#define I2C_STOPF_FLAG                      ( 1 << STOPF)
#define I2C_TC_FLAG                         ( 1 << TC)
#define I2C_BERR_FLAG                       ( 1 << BERR)
#define I2C_ARLO_FLAG                       ( 1 << ARLO)
#define I2C_OVR_FLAG                        ( 1 << OVR)
#define I2C_TIMEOUT_FLAG                    ( 1 << TIMEOUT)
#define I2C_BUSY_FLAG                       ( 1 << BUSY)

void I2C_PeripheralControl(I2C_RegDef *pI2Cx, uint8_t EnorDi);

//Peripheral Clock Setup
void I2C_PeriClockControl(I2C_RegDef *pI2Cx, uint8_t EnorDi);

//Init and Deinit
void I2C_Init(I2C_Handle *pI2CHandle);
void I2C_DeInit(I2C_RegDef *pI2Cx);

//Get status from the transmission
uint8_t I2C_GetFlagStatus(I2C_RegDef *pI2Cx , uint32_t FlagName);

//send and receive data
void I2C_MasterSendData(I2C_Handle *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t slaveAddr);

//IRQ Config and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

//Application Callback
void I2CApplicationEventCallback(I2C_Handle *pI2CHandle, uint8_t AppEv);

#endif