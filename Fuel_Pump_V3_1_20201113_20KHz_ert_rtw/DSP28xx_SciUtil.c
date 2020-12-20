/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: DSP28xx_SciUtil.c
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

#include "DSP28xx_SciUtil.h"

/* Transmit character(s) from the SCIa*/
void scia_xmit(char* pmsg, int msglen, int typeLen)
{
  int i,j,k;
  if (typeLen==1) {
    for (i = 0; i < msglen; i++) {
      while (SciaRegs.SCIFFTX.bit.TXFFST == 16) {
      }                                /* The buffer is full;*/

      SciaRegs.SCITXBUF= pmsg[i];
    }

    //while(SciaRegs.SCIFFTX.bit.TXFFST != 0){}
  } else {
    for (i = 0; i < (msglen/2); i++) {
      for (j = 0; j<=1; j++) {
        while (SciaRegs.SCIFFTX.bit.TXFFST == 16) {
        }                              /* The buffer is full;*/

        SciaRegs.SCITXBUF= pmsg[i]>>(8*j);
      }
    }

    //while(SciaRegs.SCIFFTX.bit.TXFFST != 0){}
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
