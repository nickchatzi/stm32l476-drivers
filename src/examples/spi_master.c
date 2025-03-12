/*
In this example if user presses the on-board button the master will send a packet 0xFF
Slave will check if the packet is correct, will toggle the on-board led and send back and ack.
If the ack is correct (0x65) the master's on-board LED will also toggle.
*/

#include "gpio_driver.h"
#include "spi_driver.h"
#include <string.h>

/*
PB14 -> SPI2_MISO
PB15 -> SPI2_MOSI
PB13 -> SPI2_SCLK
PB12 -> SPI2_NSS
*/

void delay(void)
{
  for (uint32_t i = 0; i<250000; i++);
}

void SPI2_GPIOInits(void)
{
  GPIO_Handle SPIPins;

  SPIPins.pGpiox = GPIOB;
  SPIPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  SPIPins.Gpio_PinConfig.GPIO_PinAltFunMode = 5;
  SPIPins.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
  SPIPins.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  SPIPins.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

  //SCL
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  GPIO_Init(&SPIPins);

  //MOSI
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
  GPIO_Init(&SPIPins);

  //MISO
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
  GPIO_Init(&SPIPins);

  //NSS
  SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
  SPI_Handle SPI2handle;
  SPI2handle.pSPIx = SPI2;
  SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
  SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
  SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
  SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
  SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;  
  
  SPI_Init(&SPI2handle);
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
    uint8_t data = 0xFF;
    uint8_t slaveResponse;

    Button_init();

    LED_Init();
  
    SPI2_GPIOInits();

    SPI2_Inits();

    SPI_SSOEConfig(SPI2, ENABLE);

    while(1)
    {
      while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13));

      delay();

      SPIPeripheralControl(SPI2,ENABLE);  

      SPI_SendData(SPI2, &data, sizeof(data));
      
      SPI_ReceiveData(SPI2, &slaveResponse, 1);

      if (slaveResponse==0x65)
      {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
      }

      while (SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

      SPIPeripheralControl(SPI2,DISABLE);

    }

}


