#ifndef STM32L4XX_H_
#define STM32L4XX_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

/********************STRUCTURE FOR GPIOx REGISTERS********************/

typedef struct 
{
    volatile uint32_t MODER;                     //Address Offset 0x00               
    volatile uint32_t OTYPER;                    //Address Offset 0x04
    volatile uint32_t OSPEEDR;                   //Address Offset 0x08
    volatile uint32_t PUPDR;                     //Address Offset 0x0C
    volatile uint32_t IDR;                       //Address Offset 0x10
    volatile uint32_t ODR;                       //Address Offset 0x14
    volatile uint32_t BSRR;                      //Address Offset 0x18
    volatile uint32_t LCKR;                      //Address Offset 0x1C
    volatile uint32_t AFR[2];                    //Address Offset ARF[0]->AFRL=0x20 & ARF[1]->AFRH=0x24

}GPIOx_RegDef;

/*********************************************************************/

/********************STRUCTURE FOR SPIx REGISTERS********************/

typedef struct 
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;

}SPI_RegDef;

/*********************************************************************/

/*********************STRUCTURE FOR RCC REGISTERS*********************/

typedef struct
{
    volatile uint32_t CR;    
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t PLLSAI1CFGR;
    volatile uint32_t PLLSAI2CFGR;
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    volatile uint32_t RES1;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RES2;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1RSTR2;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RES3;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RES4;
    volatile uint32_t APB1ENR1;
    volatile uint32_t APB1ENR2;
    volatile uint32_t APB2ENR;
    volatile uint32_t RES5;
    volatile uint32_t AHB1SMENR;
    volatile uint32_t AHB2SMENR;
    volatile uint32_t AHB3SMENR;
    volatile uint32_t RES6;
    volatile uint32_t APB1SMENR1;
    volatile uint32_t APB1SMENR2;
    volatile uint32_t APB2SMENR;
    volatile uint32_t RES7;
    volatile uint32_t CCIPR;
    volatile uint32_t RES8;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t CRRCR;
    volatile uint32_t CCIPR2; 

}RCC_RegDef;
/**********************************************************************/

/*******************STRUCTURE FOR EXTI REGISTERS*******************/

typedef struct
{
    volatile uint32_t IMR;    
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;

}EXTI_RegDef;

/**********************************************************************/

/*******************STRUCTURE FOR SYSCFG REGISTERS*******************/

typedef struct
{
    volatile uint32_t MEMRMP;    
    volatile uint32_t CFGR;
    volatile uint32_t EXTICR[4];
    volatile uint32_t SCSR;
    volatile uint32_t CGGR2;
    volatile uint32_t SWPR;
    volatile uint32_t SKR;

}SYSCFG_RegDef;

/**********************************************************************/

typedef struct
{
  volatile uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  volatile uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  volatile uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  volatile uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  volatile uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  volatile uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  volatile uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  volatile uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  volatile uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  volatile uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  volatile uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  volatile uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  volatile uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  volatile uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  volatile uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  volatile uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Registervolatile uint32_t RESERVED0[5U]; */
  volatile uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;


/******BASE ADDRESSES OF FLASH AND SRAM MEMORY******/

#define FLASH_BASEADDR                  0x08000000U
#define SRAM1_BASEADDR                  0x20000000U
#define SRAM2_BASEADDR                  0x20040000U
#define ROM_BASEADDR                    0x1FFF0000U
#define SRAM                            SRAM1_BASEADDR

/**************************************************/

/******BASE ADDRESSES OF AHBx AND APBx BUS PERIPHERAL******/

#define PERIPHERAL_BASEADDR             0x40000000U
#define APB1PERIPH_BASEADDR             PERIPHERAL_BASEADDR
#define APB2PERIPH_BASEADDR             0x40010000U
#define AHB1PERIPH_BASEADDR             0x40020000U
#define AHB2PERIPH_BASEADDR             0x48000000U

/**********************************************************/

/*******************BASE ADDRESSES OF GPIOx PERIPHERAL****************/

#define GPIOA_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR                  (AHB2PERIPH_BASEADDR + 0x2000)

/*********************************************************************/

/******************BASE ADDRESSES OF I2Cx PERIPHERAL******************/

#define I2C1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5400) 
#define I2C2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5C00)
#define I2C4_BASEADDR                   (APB1PERIPH_BASEADDR + 0x8400)

/*********************************************************************/

/******************BASE ADDRESSES OF SPIx PERIPHERAL******************/

#define SPI1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3C00)

/*********************************************************************/

/******************BASE ADDRESSES OF USARTx PERIPHERAL******************/

#define USART1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                  (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5000)

/*********************************************************************/

/*************BASE ADDRESSES OF EXT1 & SYSCFG PERIPHERAL*************/

#define EXTI_BASEADDR                   (APB2PERIPH_BASEADDR + 0x0400)
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASEADDR + 0x0000)
#define RCC_BASEADDR                    (AHB1PERIPH_BASEADDR + 0x1000)
#define SCB_BASE                        (0xE000ED00UL) 

/*********************************************************************/

/******GPIO DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT******/

#define GPIOA                           ((GPIOx_RegDef*) GPIOA_BASEADDR)                                      
#define GPIOB                           ((GPIOx_RegDef*) GPIOB_BASEADDR)
#define GPIOC                           ((GPIOx_RegDef*) GPIOC_BASEADDR) 
#define GPIOD                           ((GPIOx_RegDef*) GPIOD_BASEADDR) 
#define GPIOE                           ((GPIOx_RegDef*) GPIOE_BASEADDR) 
#define GPIOF                           ((GPIOx_RegDef*) GPIOF_BASEADDR) 
#define GPIOG                           ((GPIOx_RegDef*) GPIOG_BASEADDR) 
#define GPIOH                           ((GPIOx_RegDef*) GPIOH_BASEADDR) 
#define GPIOI                           ((GPIOx_RegDef*) GPIOI_BASEADDR)

/***********************************************************************/

/******PERIPHERAL DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT******/

#define RCC                             ((RCC_RegDef*) RCC_BASEADDR)

#define  RCC_CR_MSION                   ((uint32_t) 0x40021000) 

#define SCB                             ((SCB_Type *) SCB_BASE) 

#define EXTI                            ((EXTI_RegDef*) EXTI_BASEADDR)

#define SYSCFG                          ((SYSCFG_RegDef*) SYSCFG_BASEADDR)

/***********************************************************************/

/**********SPI DEFINITIONS BASE ADDRESSES TYPECASTED TO STRUCT**********/

#define SPI1                            ((SPI_RegDef*) SPI1_BASEADDR)                                      
#define SPI2                            ((SPI_RegDef*) SPI2_BASEADDR)
#define SPI3                            ((SPI_RegDef*) SPI3_BASEADDR) 

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS***************/

#define GPIOA_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<0) )
#define GPIOB_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<1) )
#define GPIOC_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<2) )
#define GPIOD_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<3) )
#define GPIOE_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<4) )
#define GPIOF_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<5) )
#define GPIOG_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<6) )
#define GPIOH_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<7) )
#define GPIOI_PCLK_EN()                 ( RCC->AHB2ENR |= (1<<8) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS***************/

#define I2C1_PCLK_EN()                  ( RCC->APB1ENR1 |= (1<<21) )
#define I2C2_PCLK_EN()                  ( RCC->APB1ENR1 |= (1<<22) )
#define I2C3_PCLK_EN()                  ( RCC->APB1ENR1 |= (1<<23) )
#define I2C4_PCLK_EN()                  ( RCC->APB1ENR2 |= (1<<1)  )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR SPIx PERIPHERALS***************/

#define SPI1_PCLK_EN()                  ( RCC->APB2ENR  |= (1<<12) )
#define SPI2_PCLK_EN()                  ( RCC->APB1ENR1 |= (1<<14) )
#define SPI3_PCLK_EN()                  ( RCC->APB1ENR1 |= (1<<15) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR USARTx PERIPHERALS**************/

#define USART1_PCLK_EN()                ( RCC->APB2ENR  |= (1<<14) )
#define USART2_PCLK_EN()                ( RCC->APB1ENR1 |= (1<<17) )
#define USART3_PCLK_EN()                ( RCC->APB1ENR1 |= (1<<18) )
#define UART4_PCLK_EN()                 ( RCC->APB1ENR1 |= (1<<19) )
#define UART5_PCLK_EN()                 ( RCC->APB1ENR1 |= (1<<20) )

/***********************************************************************/

/***************CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS***************/

#define SYSCFG_PCLK_EN()                ( RCC->APB2ENR |= (1<<0) )

/************************************************************************/

/***************CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS***************/

#define GPIOA_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<0) )
#define GPIOB_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<1) )
#define GPIOC_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<2) )
#define GPIOD_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<3) )
#define GPIOE_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<4) )
#define GPIOF_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<5) )
#define GPIOG_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<6) )
#define GPIOH_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<7) )
#define GPIOI_PCLK_DIS()                 ( RCC->AHB2ENR &= ~(1<<8) )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS***************/

#define I2C1_PCLK_DIS()                  ( RCC->APB1ENR1 &= ~(1<<21) )
#define I2C2_PCLK_DIS()                  ( RCC->APB1ENR1 &= ~(1<<22) )
#define I2C3_PCLK_DIS()                  ( RCC->APB1ENR1 &= ~(1<<23) )
#define I2C4_PCLK_DIS()                  ( RCC->APB1ENR2 &= ~(1<<1)  )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR SPIx PERIPHERALS***************/

#define SPI1_PCLK_DIS()                  ( RCC->APB2ENR  &= ~(1<<12) )
#define SPI2_PCLK_DIS()                  ( RCC->APB1ENR1 &= ~(1<<14) )
#define SPI3_PCLK_DIS()                  ( RCC->APB1ENR1 &= ~(1<<15) )

/***********************************************************************/

/***************CLOCK DISABLE MACROS FOR USARTx PERIPHERALS**************/

#define USART1_PCLK_DIS()                ( RCC->APB2ENR  &= ~(1<<14) )
#define USART2_PCLK_DIS()                ( RCC->APB1ENR1 &= ~(1<<17) )
#define USART3_PCLK_DIS()                ( RCC->APB1ENR1 &= ~(1<<18) )
#define UART4_PCLK_DIS()                 ( RCC->APB1ENR1 &= ~(1<<19) )
#define UART5_PCLK_DIS()                 ( RCC->APB1ENR1 &= ~(1<<20) )

/***********************************************************************/

/**************CLOCK DISABLE MACRO FOR SYSCFG PERIPHERALS**************/

#define SYSCFG_PCLK_DIS()                ( RCC->APB2ENR |= ~(1<<0) )

/***********************************************************************/

/*******************MACROS TO RESET GPIOx PERIPHERALS*******************/

#define GPIOA_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<0) ); ( RCC->AHB2RSTR &= ~(1<<0) ); } while(0)
#define GPIOB_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<1) ); ( RCC->AHB2RSTR &= ~(1<<1) ); } while(0)
#define GPIOC_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<2) ); ( RCC->AHB2RSTR &= ~(1<<2) ); } while(0)
#define GPIOD_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<3) ); ( RCC->AHB2RSTR &= ~(1<<3) ); } while(0)
#define GPIOE_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<4) ); ( RCC->AHB2RSTR &= ~(1<<4) ); } while(0)
#define GPIOF_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<5) ); ( RCC->AHB2RSTR &= ~(1<<5) ); } while(0)
#define GPIOG_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<6) ); ( RCC->AHB2RSTR &= ~(1<<6) ); } while(0)
#define GPIOH_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<7) ); ( RCC->AHB2RSTR &= ~(1<<7) ); } while(0)
#define GPIOI_REG_RESET()               do{ ( RCC->AHB2RSTR |= (1<<8) ); ( RCC->AHB2RSTR &= ~(1<<8) ); } while(0)

/***********************************************************************/

/*******************MACROS TO RESET SPIx PERIPHERALS*******************/

#define SPI1_REG_RESET()               do{ ( RCC->APB2RSTR |= (1<<12) ); ( RCC->APB2RSTR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()               do{ ( RCC->APB1RSTR1 |= (1<<14) ); ( RCC->APB1RSTR1 &= ~(1<<14) ); } while(0)
#define SPI3_REG_RESET()               do{ ( RCC->APB1RSTR1 |= (1<<15) ); ( RCC->APB1RSTR1 &= ~(1<<15) ); } while(0)

/************PORT CODE MACRO THAT RETURNS GPIOx BASE ADDRESS************/

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 :\
                                        (x == GPIOB) ? 1 :\
                                        (x == GPIOC) ? 2 :\
                                        (x == GPIOD) ? 3 :\
                                        (x == GPIOE) ? 4 :\
                                        (x == GPIOF) ? 5 :\
                                        (x == GPIOG) ? 6 :\
                                        (x == GPIOH) ? 7 :\
                                        (x == GPIOI) ? 8 :0 )
/***********************************************************************/

/*******************IRQ NUMBERS MACROS FOR EXTIx LINES******************/

//GPIO EXTI IRQ NUMBERS
#define IRQ_NO_EXTI0                    6
#define IRQ_NO_EXTI1                    7
#define IRQ_NO_EXTI2                    8
#define IRQ_NO_EXTI3                    9
#define IRQ_NO_EXTI4                    10
#define IRQ_NO_EXTI9_5                  23
#define IRQ_NO_EXTI15_10                40

//SPI IRQ NUMBERS
#define IRQ_NO_SPI1                     35
#define IRQ_NO_SPI2                     36
#define IRQ_NO_SPI3                     51

/***********************************************************************/

/***********************PROCESSOR SPECIFIC DETAILS**********************/

/*
ARM Cortex M4 Processor NVIC ISERx register Addresses
*/

#define NVIC_ISER0                      ( (volatile uint32_t*) 0xE000E100 )
#define NVIC_ISER1                      ( (volatile uint32_t*) 0xE000E104 )
#define NVIC_ISER2                      ( (volatile uint32_t*) 0xE000E108 )
#define NVIC_ISER3                      ( (volatile uint32_t*) 0xE000E10C )

/*
ARM Cortex M4 Processor NVIC ICERx register Addresses
*/

#define NVIC_ICER0                      ( (volatile uint32_t*) 0xE000E180 )
#define NVIC_ICER1                      ( (volatile uint32_t*) 0xE000E184 )
#define NVIC_ICER2                      ( (volatile uint32_t*) 0xE000E188 )
#define NVIC_ICER3                      ( (volatile uint32_t*) 0xE000E18C )

/*
ARM Cortex M4 Processor NVIC Priority register Address
*/

#define NVIC_PR_BASE_ADDR               ( (volatile uint32_t*) 0xE000E400 ) 

#define NO_PR_BITS_IMPLEMENTED          4

/***********************************************************************/

/*****************************GENERIC MACROS****************************/

#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define RGPIO_PIN_RESET                 RESET
#define FLAG_SET 			            SET
#define FLAG_RESET                      RESET
#define __FPU_PRESENT                   1U 
#define __FPU_USED                      0U
#define FALSE                           0
#define TRUE                            1

/***********************************************************************/

#endif