#include "gpio_driver.h"
#include "spi_driver.h"
#include "rcc_driver.h"
#include "lcd.h"
#include <string.h>

/*
PB14 -> SPI2_MISO
PB15 -> SPI2_MOSI
PB13 -> SPI2_SCLK
PB12 -> SPI2_NSS
*/

RCC_Handle RCCHandle;


void delay(void)
{
  for (uint32_t i = 0; i<300000; i++);
}

void HSI_Init(void)
{
  RCC_Handle rcc;
  rcc.pRCC = RCC;

  rcc.RCC_Config.RCC_AHB_Presc = AHB_NO_DIV;
  rcc.RCC_Config.RCC_APB1 = APB1_NO_DIV;
  rcc.RCC_Config.RCC_APB2 = APB2_NO_DIV;

  setHSIclock(&rcc);
}

void SPI2_GPIOInits(void)
{
  GPIO_Handle SPIPins;

  SPIPins.pGpiox = GPIOB;
  SPIPins.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
  SPIPins.Gpio_PinConfig.GPIO_PinAltFunMode = 7;
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
  //SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
  //GPIO_Init(&SPIPins);

  //NSS
  //SPIPins.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  //GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
  SPI_Handle SPI2handle;
  SPI2handle.pSPIx = SPI2;
  SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
  SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
  SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
  SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
  SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;  
  
  SPI_Init(&SPI2handle);
}

void RSTPIN_Init()
{
  GPIO_Handle GpioRst;

  GpioRst.pGpiox = GPIOA;
  GpioRst.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
  GpioRst.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioRst.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioRst.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioRst.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioRst);
}

void RSPIN_Init()
{
  GPIO_Handle GpioRs;

  GpioRs.pGpiox = GPIOA;
  GpioRs.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
  GpioRs.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioRs.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioRs.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioRs.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioRs);
}

void CSPIN_Init()
{
  GPIO_Handle GpioCS;

  GpioCS.pGpiox = GPIOB;
  GpioCS.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  GpioCS.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
  GpioCS.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  GpioCS.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  GpioCS.Gpio_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

  GPIO_Init(&GpioCS);
}

void ButtonUp_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOA;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

  GPIO_Init(&button);
}

void ButtonEnter_init(void)
{
  GPIO_Handle button;

  button.pGpiox = GPIOA;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

  GPIO_Init(&button);
}

void ButtonDown_init(void)
{
  GPIO_Handle button;
  memset(&button,0,sizeof(button));

  button.pGpiox = GPIOC;
  button.Gpio_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
  button.Gpio_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
  button.Gpio_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
  button.Gpio_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

  GPIO_Init(&button);
}

int main(void)
{
  char menu[] = "Menu";
  char line[] = "---------------------";
  char arrow[] = ">";
  char option1[] = "Option 1";
  char option2[] = "Option 2";
  char option3[] = "Option 3";
  char option4[] = "Option 4";
  char option5[] = "Option 5";
  char option6[] = "Option 6";

  HSI_Init();

  RSTPIN_Init();
  RSPIN_Init();
  CSPIN_Init();
  ButtonUp_init();
  ButtonEnter_init(); 
  ButtonDown_init();
  SPI2_GPIOInits();
  SPI2_Inits();

  GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

  SPI_SSIConfig(SPI2, ENABLE);
  SPIPeripheralControl(SPI2, ENABLE);

  lcd_init();
  lcd_set_black_background();

  lcd_set_page(0);
  lcd_set_column(50);
  
  for (int i = 0; i < strlen(menu); i++) {
    lcd_write_char(menu[i]);
  }

  lcd_set_page(1);
  lcd_set_column(1);
  
  for (int i = 0; i < strlen(line); i++) {
    lcd_write_char(line[i]);
  }

  lcd_set_page(2);
  lcd_set_column(1);
  
  for (int i = 0; i < strlen(arrow); i++) {
    lcd_write_char(arrow[i]);
  }

  lcd_set_page(2);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option1); i++) {
    lcd_write_char(option1[i]);
  }

  lcd_set_page(3);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option2); i++) {
    lcd_write_char(option2[i]);
  }

  lcd_set_page(4);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option3); i++) {
    lcd_write_char(option3[i]);
  }

  lcd_set_page(5);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option4); i++) {
    lcd_write_char(option4[i]);
  }

  lcd_set_page(6);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option5); i++) {
    lcd_write_char(option5[i]);
  }

  lcd_set_page(7);
  lcd_set_column(10);

  for (int i = 0; i < strlen(option6); i++) {
    lcd_write_char(option6[i]);
  }
    while(1);
}

void EXTI15_10_IRQHandler()
{
    GPIO_IRQHandling(GPIO_PIN_NO_13);

    static uint8_t selected_option = 2;  // Start from option 1 (page 2)

    // Remove old arrow
    lcd_set_page(selected_option);
    lcd_set_column(1);
    lcd_write_char(' ');  // overwrite old arrow with a space

    // Move to next option
    selected_option++;
    if (selected_option > 7) {
        selected_option = 2;  // Loop back to option 1 
    }

    // Draw new arrow
    lcd_set_page(selected_option);
    lcd_set_column(1);
    lcd_write_char('>');
    delay();
}
