
#include "gpio_driver.h"
#include "usart_driver.h"
#include <string.h>

/*
PA2 -> TX
PA3 -> RX
*/

USART_Handle USART2Handle;

void delay(void)
{
  for (uint32_t i = 0; i<250000; i++);
}

void USART_GPIOInits(void)
{
  GPIO_Handle USARTPins;
  
  USARTPins.pGpiox = GPIOA;
  USARTPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  USARTPins.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  USARTPins.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  USARTPins.Gpio_PinConfig.GPIO_PinAltFunMode = 7;
  USARTPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  //TX
  USARTPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
  GPIO_Init(&USARTPins);

  //RX
  USARTPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
  GPIO_Init(&USARTPins);
}

void USART_Inits(void)
{
USART2Handle.pUSARTx = USART1;
USART2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
USART2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
USART2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
//USART2Handle.USART_Config.USART_Oversampling = USART_OVER8;
USART2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
USART2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
USART2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

USART_Init(&USART2Handle);
}

void Button_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOC;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

  GPIO_Init(&button);
}

void LED_Init()
{
  GPIO_Handle GpioLed;

  GpioLed.pGpiox = GPIOA;
  GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioLed);
}

int main (void)
{
    uint8_t receivedData = 0;

    uint8_t Data = 0x60;

    Button_init();

    LED_Init();

    USART_GPIOInits();

    USART_Inits();

    USART_PeripheralControl(USART1, ENABLE);

    delay();

    USART_SendData(&USART2Handle, &Data, sizeof(Data));
  
    while(1)
    {
      
      USART_ReceiveData(&USART2Handle, &receivedData, sizeof(receivedData));

      delay();

      if (receivedData == 0x20)
      {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

        delay();
      }
    } 
}
