#include "c2000BoardSupport.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "DSP2833x_GlobalPrototypes.h"
#include "rtwtypes.h"
#include "Fuel_Pump_V3_1_20201113_20KHz.h"
#include "Fuel_Pump_V3_1_20201113_20KHz_private.h"

void init_I2C_GPIO(void)
{
  EALLOW;                              /* Initial I2C GPIO pin*/
  GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;  /* Enable pull-up for GPIO32 (SDAA)*/
  GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;  /* Enable pull-up for GPIO33 (SCLA)*/
  GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1; /* Configure GPIO32 for SDAA operation*/
  GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1; /* Configure GPIO33 for SCLA operation*/
  EDIS;
}

void init_I2C_A(void)
{
  /* Initialize I2C*/
  EALLOW;
  EDIS;
  I2caRegs.I2CMDR.bit.MST = 1;         /* Select master or slave mode*/
  I2caRegs.I2CMDR.bit.DLB = 0;         /* Enable digital loopback bit */
  I2caRegs.I2CPSC.all = 8;          /* Prescaler - need 7-12 Mhz on module clk*/
  I2caRegs.I2CCLKL = 4;                /* NOTE: must be non zero*/
  I2caRegs.I2CCLKH = 4;                /* NOTE: must be non zero*/
  I2caRegs.I2CFFTX.all |= 0x6000;      /* Enable TxFIFO mode*/
  I2caRegs.I2CFFRX.all |= 0x2000;      /* Enable RxFIFO mode*/
  I2caRegs.I2CIER.bit.ARDY = 1;      /* Enable register-access-ready interrupt*/
  I2caRegs.I2CMDR.bit.IRS = 1;         /* Take I2C out of reset*/
}
