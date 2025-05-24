/*
This example writes and the reads to address 0x1200 from 24LC256 EEPROM chip the value 0x60.
Chip select bits are A2=0, A1=0, A0=1
*/

#include "gpio_driver.h"
#include "i2c_driver.h"
#include "rcc_driver.h"

/*
PB6 -> I2C_SCL
PB9 -> I2X SDA
*/
#define MY_ADDRESS  0x60
#define SLAVE_ADDRESS   0x51

RCC_Handle RCCHandle;
I2C_Handle I2C1Handle;

uint8_t rcv_buf[32];


void delay(void)
{
  for (volatile uint32_t i = 0; i<10000; i++);
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
I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDRESS;
I2C1Handle.I2C_Config.I2C_Freq = I2C_BUS_100KHZ;
I2C1Handle.I2C_Config.I2C_Mode = I2C_SCL_SPEED_SM;
//I2C1Handle.I2C_Config.I2C_NoStretch = I2C_DISABLE_NO_STRETCH;
//I2C1Handle.I2C_Config.I2C_OwnAddressMode = I2C_10BIT_ADDRESS_MODE;
I2C1Handle.I2C_Config.I2C_SlaveAddressMode = I2C_7BIT_ADDRESS_MODE;

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
    uint8_t dataAddress[2] = {0x12,0x00};

    uint8_t data[3] = {0x12,0x00 ,0x60};

    uint8_t received_data[1];

    uint8_t prev_button_state = 1;
   
    HSI_Init();

    Button_init();

    LED_Init();

    I2C1_GPIOInits();

    I2C_Inits();

    I2C_PeripheralControl(I2C1, ENABLE);

    while (1)
    {
        uint8_t curr_button_state = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);

        // Detect falling edge: not pressed -> pressed
        if (prev_button_state == 1 && curr_button_state == 0)
        {
            delay(); 

            if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
            {
                I2C_MasterSendData(&I2C1Handle, data, 3, SLAVE_ADDRESS, I2C_DISABLE_SR);

                delay();

                I2C_MasterSendData(&I2C1Handle, dataAddress, 2, SLAVE_ADDRESS, I2C_ENABLE_SR);

                I2C_MasterReceiveData(&I2C1Handle, received_data, 1, SLAVE_ADDRESS, I2C_DISABLE_SR);

                if (received_data[0] == 0x60)
                {
                    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
                }

            }
        }

        prev_button_state = curr_button_state;
    }
}