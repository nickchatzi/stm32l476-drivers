/*
In this example, slave will wait for master to send a data packet.
If the packet is correct (0xFF), then slave will send back an ack response(0x65) and toggle the on-board LED.
Master will read the ack and if correct will also toggle the on-board LED.
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
  SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
  SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
  SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
  SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
  SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
  SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;  
  
  SPI_Init(&SPI2handle);
}

void LED_Init(void)
{
  GPIO_Handle GpioLed;

  GpioLed.pGpiox = GPIOD;
  GpioLed.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioLed.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioLed.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioLed.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioLed.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioLed);
}

int main (void)
{
  uint8_t data;
  uint8_t ack = 0x65;

  SPI2_GPIOInits();

  SPI2_Inits();

  LED_Init();

  SPIPeripheralControl(SPI2,ENABLE);
  
  while(1)
  {
    SPI_ReceiveData(SPI2, &data, sizeof(data));

    if (data==0xFF)
    {
      SPI_SendData(SPI2, &ack, 1);

      GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
    } 
  }
}


