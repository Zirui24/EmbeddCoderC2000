/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ert_main.c
 *
 * Code generated for Simulink model 'Fuel_Pump_V3_1_20201113_20KHz'.
 *
 * Model version                  : 1.40
 * Simulink Coder version         : 9.2 (R2019b) 18-Jul-2019
 * C/C++ source code generated on : Tue Nov 17 11:10:34 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Texas Instruments->C2000
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "Fuel_Pump_V3_1_20201113_20KHz.h"
#include "rtwtypes.h"

volatile int IsrOverrun = 0;
boolean_T isRateRunning[4] = { 0, 0, 0, 0 };

boolean_T need2runFlags[4] = { 0, 0, 0, 0 };

void rt_OneStep(void)
{
  boolean_T eventFlags[4];
  int_T i;

  /* Check base rate for overrun */
  if (isRateRunning[0]++) {
    IsrOverrun = 1;
    isRateRunning[0]--;                /* allow future iterations to succeed*/
    return;
  }

  /*
   * For a bare-board target (i.e., no operating system), the rates
   * that execute this base step are buffered locally to allow for
   * overlapping preemption.  The generated code includes function
   * writeCodeInfoFcn() which sets the rates
   * that need to run this time step.  The return values are 1 and 0
   * for true and false, respectively.
   */
  Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep(eventFlags);
  enableTimer0Interrupt();
  Fuel_Pump_V3_1_20201113_20KHz_step0();

  /* Get model outputs here */
  disableTimer0Interrupt();
  isRateRunning[0]--;
  for (i = 1; i < 4; i++) {
    if (eventFlags[i]) {
      if (need2runFlags[i]++) {
        IsrOverrun = 1;
        need2runFlags[i]--;            /* allow future iterations to succeed*/
        break;
      }
    }
  }

  for (i = 1; i < 4; i++) {
    if (isRateRunning[i]) {
      /* Yield to higher priority*/
      return;
    }

    if (need2runFlags[i]) {
      isRateRunning[i]++;
      enableTimer0Interrupt();

      /* Step the model for subrate "i" */
      switch (i)
      {
       case 1 :
        Fuel_Pump_V3_1_20201113_20KHz_step1();

        /* Get model outputs here */
        break;

       case 2 :
        Fuel_Pump_V3_1_20201113_20KHz_step2();

        /* Get model outputs here */
        break;

       case 3 :
        Fuel_Pump_V3_1_20201113_20KHz_step3();

        /* Get model outputs here */
        break;

       default :
        break;
      }

      disableTimer0Interrupt();
      need2runFlags[i]--;
      isRateRunning[i]--;
    }
  }
}

volatile boolean_T stopRequested = false;
volatile boolean_T runModel = false;
int main(void)
{
  float modelBaseRate = 0.001;
  float systemClock = 150;

  /* Initialize variables */
  stopRequested = false;
  runModel = false;
  c2000_flash_init();
  init_board();

#ifdef MW_EXEC_PROFILER_ON

  config_profilerTimer();

#endif

  ;
  rtmSetErrorStatus(Fuel_Pump_V3_1_20201113_20KH_M, 0);
  Fuel_Pump_V3_1_20201113_20KHz_initialize();
  configureTimer0(modelBaseRate, systemClock);
  runModel =
    rtmGetErrorStatus(Fuel_Pump_V3_1_20201113_20KH_M) == (NULL);
  enableTimer0Interrupt();
  enable_interrupts();
  globalInterruptEnable();
  while (runModel) {
    stopRequested = !(
                      rtmGetErrorStatus(Fuel_Pump_V3_1_20201113_20KH_M) == (NULL));
    idletask_num1();
  }

  /* Disable rt_OneStep() here */

  /* Terminate model */
  Fuel_Pump_V3_1_20201113_20KHz_terminate();
  globalInterruptDisable();
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
