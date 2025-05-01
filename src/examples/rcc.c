#include "rcc_driver.h"

RCC_Handle rcc;

void MSI_init(void)
{

  rcc.RCC_Config.RCC_MSIRangeStandby = MSI_CLOCK_4MHZ;
  rcc.RCC_Config.RCC_MSIRange = MSI_CLOCK_32MHZ;

  setMSIclock(&rcc);
}

int main(void)
{
    setMSIclock(&rcc);
}
