/*

*/

#include "gpio_driver.h"
#include "i2c_driver.h"
#include "rcc_driver.h"
#include <string.h>

/*
PB6 -> I2C_SCL
PB9 -> I2X SDA
*/
#define MY_ADDRESS  0X61
#define SLAVE_ADDRESS   0X11

RCC_Handle RCCHandle;
I2C_Handle I2C1Handle;

void delay(void)
{
  for (uint32_t i = 0; i<250000; i++);
}

void I2C1_GPIOInits(void)
{
  GPIO_Handle I2CPins;
  
  I2CPins.pGpiox = GPIOB;
  I2CPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  I2CPins.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
  I2CPins.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
  I2CPins.Gpio_PinConfig.GPIO_PinAltFunMode = 4;
  I2CPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  //SCL
  I2CPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
  GPIO_Init(&I2CPins);

  //SDA
  I2CPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
  GPIO_Init(&I2CPins);
}

void I2C_Inits(void)
{
I2C1Handle.pI2Cx = I2C1;
//I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
I2C1Handle.I2C_Config.I2C_Freq = I2C_BUS_100KHZ;
I2C1Handle.I2C_Config.I2C_Mode = I2C_SCL_SPEED_SM;

I2C_Init(&I2C1Handle);
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

    uint8_t data[] = "Hello World!";


    Button_init();

    LED_Init();

    I2C1_GPIOInits();

    I2C_Inits();

    I2C_PeripheralControl(I2C1, ENABLE);

    while(1)
    {
        while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

        delay(); 
  
        I2C_MasterSendData(&I2C1Handle,data,strlen((char*)data),SLAVE_ADDRESS, I2C_DISABLE_SR);
    }

}


