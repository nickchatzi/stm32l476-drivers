#include "stm32l476.h"
#include "gpio_driver.h"
#include <string.h>

//void EXTI15_10_IRQHandler (void);


void delay(void)
{
  for (uint32_t i = 0; i<250000; i++);
}

int main (void)
{
  GPIO_Handle GpioLed;
  GPIO_Handle button;

  memset(&GpioLed,0,sizeof(GpioLed));
  memset(&button,0,sizeof(button));

  GpioLed.pGpiox = GPIOA;
  GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioLed);

  button.pGpiox = GPIOC;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

  GPIO_Init(&button);

  //GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
  //GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
  

  while(1)
  {
    if (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) == 0)
    {
      GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
      delay();
    }
  }
    //return 0;
}