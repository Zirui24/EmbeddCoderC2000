#include "c2000BoardSupport.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "DSP2833x_GlobalPrototypes.h"
#include "rtwtypes.h"
#include "Fuel_Pump_V3_1_20201113_20KHz.h"
#include "Fuel_Pump_V3_1_20201113_20KHz_private.h"

void enableExtInterrupt (void);
void disableWatchdog(void)
{
  int *WatchdogWDCR = (void *) 0x7029;
  asm(" EALLOW ");
  *WatchdogWDCR = 0x0068;
  asm(" EDIS ");
}

interrupt void ADCINT_isr(void)
{
  isr_int1pie6_task_fcn();
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
  AdcRegs.ADCST.bit.INT_SEQ2_CLR = 1;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
                                     /* Acknowledge to receive more interrupts*/
}

interrupt void I2CINT1A_isr(void)
{
  isr_int8pie1_task_fcn();
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
                                     /* Acknowledge to receive more interrupts*/
}

void idletask_num1(void)
{
  idle_num1_task_fcn();
}

void enable_interrupts()
{
  EALLOW;
  PieVectTable.ADCINT = &ADCINT_isr;   /* Hook interrupt to the ISR*/
  EDIS;
  PieCtrlRegs.PIEIER1.bit.INTx6 = 1;   /* Enable interrupt ADCINT*/
  IER |= M_INT1;
  EALLOW;
  PieVectTable.I2CINT1A = &I2CINT1A_isr;/* Hook interrupt to the ISR*/
  EDIS;
  PieCtrlRegs.PIEIER8.bit.INTx1 = 1;   /* Enable interrupt I2CINT1A*/
  IER |= M_INT8;

  /* Enable global Interrupts and higher priority real-time debug events:*/
  EINT;                                /* Enable Global interrupt INTM*/
  ERTM;                               /* Enable Global realtime interrupt DBGM*/
}

void init_SCI(void)
{                                      /* initialize SCI & FIFO registers */
  EALLOW;

  /*
   * Initialize SCI_A with following parameters:
   *    BaudRate              : 115031
   *    CharacterLengthBits   : 8
   *    EnableLoopBack        : 0
   *    NumberOfStopBits      : 1
   *    ParityMode            : Even
   *    SuspensionMode        : Free_run
   *    CommMode              : Raw_data
   */
  SciaRegs.SCICCR.bit.STOPBITS = 0;
                    /*Number of stop bits. (0: One stop bit, 1: Two stop bits)*/
  SciaRegs.SCICCR.bit.PARITY = 1;/*Parity mode (0: Odd parity, 1: Even parity)*/
  SciaRegs.SCICCR.bit.PARITYENA = 1;   /*Enable Pary Mode */
  SciaRegs.SCICCR.bit.LOOPBKENA = 0;   /*Loop Back enable*/
  SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;/*ADDR/IDLE Mode control*/
  SciaRegs.SCICCR.bit.SCICHAR = 7;     /*Character length*/
  SciaRegs.SCICTL1.bit.RXERRINTENA = 0;/*Disable receive error interrupt*/
  SciaRegs.SCICTL1.bit.SWRESET = 1;    /*Software reset*/
  SciaRegs.SCICTL1.bit.TXENA = 1;      /* SCI transmitter enable*/
  SciaRegs.SCICTL1.bit.RXENA = 0;      /* SCI receiver enable*/
  SciaRegs.SCIHBAUD = 0U;
  SciaRegs.SCILBAUD = 162U;

  /*Free run, continue SCI operation regardless of suspend*/
  SciaRegs.SCIPRI.bit.FREE = 1;        /* Free emulation mode control*/
  SciaRegs.SCIPRI.bit.SOFT = 0;        /* Interrupt priority select*/
  SciaRegs.SCIFFCT.bit.ABDCLR = 0;
  SciaRegs.SCIFFCT.bit.CDC = 0;
  SciaRegs.SCIFFTX.bit.SCIRST = 1;     /* SCI reset rx/tx channels*/
  SciaRegs.SCIFFTX.bit.SCIFFENA = 1;   /* SCI FIFO enhancements are enabled.*/
  SciaRegs.SCIFFTX.bit.TXFIFOXRESET = 1;/* Re-enable transmit FIFO operation.*/
  SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;/* Re-enable receive FIFO operation.*/
  EDIS;
}

void init_SCI_GPIO(void)
{
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;  /*Enable pull-up for GPIO29*/
  GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1; /*Configure GPIO29 as SCITXDA*/
  EDIS;
}

void configureGPIOExtInterrupt (void)
{
}

void enableExtInterrupt (void)
{
}
