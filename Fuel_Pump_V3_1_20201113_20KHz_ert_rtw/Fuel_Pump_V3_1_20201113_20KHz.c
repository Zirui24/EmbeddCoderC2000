/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Fuel_Pump_V3_1_20201113_20KHz.c
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
#include "Fuel_Pump_V3_1_20201113_20KHz_private.h"
#define Fue_IN_Temporarily_Disconnected (3U)
#define Fue_SLAVE_ID_DATA_TYPE_NOT_USED (0U)
#define Fuel_P_RESOURCE_PROTECTION_MASK (0U)
#define Fuel_P_SLAVE_ID_LENGTH_NOT_USED (0U)
#define Fuel_Pump_MTA_ADDRESS_EXTENSION (0U)
#define Fuel_Pump_V3_1_202_IN_Connected (1U)
#define Fuel_Pump_V3_1__IN_Disconnected (2U)
#define Fuel_Pump_V3_IN_NO_ACTIVE_CHILD (0U)
#define Fuel_Pump_V3_S_STATUS_QUALIFIER (0U)
#define Fuel_Pump__PERMANENT_DISCONNECT (1U)
#define Fuel_Pump__TEMPORARY_DISCONNECT (0U)
#define Fuel_RESOURCE_AVAILABILITY_MASK (66U)

/* Exported block states */
real32_T Data_Buffer_ThetaSMO[1000];   /* '<Root>/Data Store Memory100' */
real32_T Buffer_Out_ThetaSMO;          /* '<Root>/Data Store Memory102' */
real32_T Buffer_Out_ThetaAD2S;         /* '<Root>/Data Store Memory111' */
real32_T Buffer_Out_IqDem;             /* '<Root>/Data Store Memory114' */
real32_T Buffer_Out_Iq;                /* '<Root>/Data Store Memory115' */
real32_T Buffer_Out_Id;                /* '<Root>/Data Store Memory116' */
real32_T Buffer_Out_IdDem;             /* '<Root>/Data Store Memory117' */
real32_T n_R;                          /* '<Root>/Data Store Memory118' */
real32_T n_Q;                          /* '<Root>/Data Store Memory119' */
real32_T Ready_1210;                   /* '<Root>/Data Store Memory12' */
real32_T Current_W1;                   /* '<Root>/Data Store Memory14' */
real32_T Current_U1;                   /* '<Root>/Data Store Memory17' */
real32_T Current_V1;                   /* '<Root>/Data Store Memory18' */
real32_T Angle;                        /* '<Root>/Data Store Memory20' */
real32_T SMO_A;                        /* '<Root>/Data Store Memory22' */
real32_T SMO_K1;                       /* '<Root>/Data Store Memory23' */
real32_T SMO_K2;                       /* '<Root>/Data Store Memory24' */
real32_T nDem_PI;                      /* '<Root>/Data Store Memory27' */
real32_T Uq_VVF;                       /* '<Root>/Data Store Memory28' */
real32_T Ud_VVF;                       /* '<Root>/Data Store Memory29' */
real32_T LWFf;                         /* '<Root>/Data Store Memory34' */
real32_T n_Filter;                     /* '<Root>/Data Store Memory35' */
real32_T Us;                           /* '<Root>/Data Store Memory36' */
real32_T I_alpha;                      /* '<Root>/Data Store Memory46' */
real32_T I_beta;                       /* '<Root>/Data Store Memory48' */
real32_T SMO_KP;                       /* '<Root>/Data Store Memory49' */
real32_T SMO_KI;                       /* '<Root>/Data Store Memory50' */
real32_T V_alpha;                      /* '<Root>/Data Store Memory51' */
real32_T V_beta;                       /* '<Root>/Data Store Memory52' */
real32_T EMF_alpha;                    /* '<Root>/Data Store Memory53' */
real32_T EMF_beta;                     /* '<Root>/Data Store Memory54' */
real32_T SMO_uin;                      /* '<Root>/Data Store Memory55' */
real32_T LWFDem;                       /* '<Root>/Data Store Memory56' */
real32_T nDem;                         /* '<Root>/Data Store Memory57' */
real32_T n;                            /* '<Root>/Data Store Memory58' */
real32_T SetpointValue;                /* '<Root>/Data Store Memory59' */
real32_T AD_Angle;                     /* '<Root>/Data Store Memory6' */
real32_T n_Step;                       /* '<Root>/Data Store Memory60' */
real32_T SMO_Theta;                    /* '<Root>/Data Store Memory63' */
real32_T SMO_n;                        /* '<Root>/Data Store Memory64' */
real32_T TEMP_MCU1;                    /* '<Root>/Data Store Memory65' */
real32_T TEMP_MCU2;                    /* '<Root>/Data Store Memory66' */
real32_T TEMP_MOT1;                    /* '<Root>/Data Store Memory67' */
real32_T TEMP_MOT2;                    /* '<Root>/Data Store Memory68' */
real32_T en;                           /* '<Root>/Data Store Memory71' */
real32_T uin;                          /* '<Root>/Data Store Memory72' */
real32_T un_PI;                        /* '<Root>/Data Store Memory73' */
real32_T PK_Kpn;                       /* '<Root>/Data Store Memory74' */
real32_T SMO_Slide;                    /* '<Root>/Data Store Memory75' */
real32_T SMO_ComP;                     /* '<Root>/Data Store Memory76' */
real32_T n_1210;                       /* '<Root>/Data Store Memory77' */
real32_T PK_Kin;                       /* '<Root>/Data Store Memory78' */
real32_T SMO_Filter;                   /* '<Root>/Data Store Memory79' */
real32_T Data_Buffer_ThetaAD2S[1000];  /* '<Root>/Data Store Memory80' */
real32_T IqDem;                        /* '<Root>/Data Store Memory81' */
real32_T Id;                           /* '<Root>/Data Store Memory82' */
real32_T Data_Buffer_IqDem[1000];      /* '<Root>/Data Store Memory83' */
real32_T uiId;                         /* '<Root>/Data Store Memory84' */
real32_T Ud;                           /* '<Root>/Data Store Memory86' */
real32_T IdDem;                        /* '<Root>/Data Store Memory87' */
real32_T Iq;                           /* '<Root>/Data Store Memory88' */
real32_T Data_Buffer_Iq[1000];         /* '<Root>/Data Store Memory89' */
real32_T uiIq;                         /* '<Root>/Data Store Memory90' */
real32_T PK_KpIq;                      /* '<Root>/Data Store Memory92' */
real32_T Uq;                           /* '<Root>/Data Store Memory93' */
real32_T PK_KpId;                      /* '<Root>/Data Store Memory94' */
real32_T PK_KiId;                      /* '<Root>/Data Store Memory95' */
real32_T PK_KiIq;                      /* '<Root>/Data Store Memory96' */
real32_T Data_Buffer_IdDem[1000];      /* '<Root>/Data Store Memory97' */
real32_T Data_Buffer_Id[1000];         /* '<Root>/Data Store Memory98' */
real32_T eLWFf;                        /* '<Root>/Data Store Memory99' */
uint32_T CPU_Timer;                    /* '<Root>/Data Store Memory112' */
int32_T VVF_Frq;                       /* '<Root>/Data Store Memory4' */
int32_T IF_Frq;                        /* '<Root>/Data Store Memory47' */
uint16_T ControlMode;                  /* '<Root>/Data Store Memory' */
uint16_T Mid_Position;                 /* '<Root>/Data Store Memory1' */
uint16_T Buffer_Flag;                  /* '<Root>/Data Store Memory101' */
uint16_T bFlt_OvrLim_WindingUT;        /* '<Root>/Data Store Memory103' */
uint16_T bFlt_OvrLim_WindingVT;        /* '<Root>/Data Store Memory104' */
uint16_T bFlt_OvrLim_TempMOST;         /* '<Root>/Data Store Memory105' */
uint16_T bFlt_OvrLim_TempUT;           /* '<Root>/Data Store Memory106' */
uint16_T bFlt_OvrLim_TempVT;           /* '<Root>/Data Store Memory107' */
uint16_T MCU_ST;                       /* '<Root>/Data Store Memory108' */
uint16_T MCU_IN;                       /* '<Root>/Data Store Memory109' */
uint16_T Offset_W1;                    /* '<Root>/Data Store Memory11' */
uint16_T GF_MODE;                      /* '<Root>/Data Store Memory110' */
uint16_T SCI_Break_Error;              /* '<Root>/Data Store Memory113' */
uint16_T TX_DATA;                      /* '<Root>/Data Store Memory16' */
uint16_T Stop;                         /* '<Root>/Data Store Memory19' */
uint16_T U1;                           /* '<Root>/Data Store Memory2' */
uint16_T RMS_Count;                    /* '<Root>/Data Store Memory21' */
uint16_T RX_DATA;                      /* '<Root>/Data Store Memory25' */
uint16_T V1;                           /* '<Root>/Data Store Memory3' */
uint16_T Protect_Flag;                 /* '<Root>/Data Store Memory30' */
uint16_T SCI_Error;                    /* '<Root>/Data Store Memory31' */
uint16_T NC1;                          /* '<Root>/Data Store Memory32' */
uint16_T NC2;                          /* '<Root>/Data Store Memory37' */
uint16_T bFlt_LWFf;                    /* '<Root>/Data Store Memory38' */
uint16_T bFlt_Resolver;                /* '<Root>/Data Store Memory39' */
uint16_T bFlt_MOST_T;                  /* '<Root>/Data Store Memory40' */
uint16_T bFlt_WindingUT_Sensor;        /* '<Root>/Data Store Memory41' */
uint16_T bFlt_WindingVT_Sensor;        /* '<Root>/Data Store Memory42' */
uint16_T bFlt_Winding_I;               /* '<Root>/Data Store Memory43' */
uint16_T bFlt_OvrLim_I;                /* '<Root>/Data Store Memory44' */
uint16_T bFlt_OvrLim_MOST;             /* '<Root>/Data Store Memory45' */
uint16_T W1;                           /* '<Root>/Data Store Memory5' */
uint16_T ATEMP_MCU1;                   /* '<Root>/Data Store Memory61' */
uint16_T ATEMP_MCU2;                   /* '<Root>/Data Store Memory62' */
uint16_T TEMP_Flag;                    /* '<Root>/Data Store Memory69' */
uint16_T SMO_Flag;                     /* '<Root>/Data Store Memory70' */
uint16_T Offset_U1;                    /* '<Root>/Data Store Memory8' */
uint16_T Offset_V1;                    /* '<Root>/Data Store Memory9' */

/* Block signals (default storage) */
B_Fuel_Pump_V3_1_20201113_20K_T Fuel_Pump_V3_1_20201113_20KHz_B;

/* Block states (default storage) */
DW_Fuel_Pump_V3_1_20201113_20_T Fuel_Pump_V3_1_20201113_20KH_DW;

/* Real-time model */
RT_MODEL_Fuel_Pump_V3_1_20201_T Fuel_Pump_V3_1_20201113_20KH_M_;
RT_MODEL_Fuel_Pump_V3_1_20201_T *const Fuel_Pump_V3_1_20201113_20KH_M =
  &Fuel_Pump_V3_1_20201113_20KH_M_;

/* Forward declaration for local functions */
static void Fuel_Pump_V3_SystemCore_release(dsp_simulink_MovingAverage_Fu_T *obj);
static void Fuel_Pump_V3__SystemCore_delete(dsp_simulink_MovingAverage_Fu_T *obj);
static void matlabCodegenHandle_matlabCodeg(dsp_simulink_MovingAverage_Fu_T *obj);

/* Forward declaration for local functions */
static real32_T Fuel_Pump_V3_1_20201113_20K_mod(real32_T x);
static void Fuel_Pump__SystemCore_release_c(const
  codertarget_tic2000_blocks_SP_T *obj);
static void Fuel_Pump_V_SystemCore_delete_h(const
  codertarget_tic2000_blocks_SP_T *obj);
static void matlabCodegenHandle_matlabCod_e(codertarget_tic2000_blocks_SP_T *obj);

/* Forward declaration for local functions */
static void Fuel_Pump_V3_1_20201113_20_init(void);
static void Fuel_Pump_V3_1_2020111_get_mta0(uint16_T first_byte);
static void Fuel_Pump_V3_1_20201113__tx_dto(uint16_T tx_comm_type);
static void Fuel_Pump_V3__unhandled_command(void);
static void Fuel_Pump_V3_1_202011_write_daq(void);
static void Fuel_Pump_V3_1_20201113_set_mta(void);
static void rate_monotonic_scheduler(void);
static uint16_T adcInitFlag = 0;
real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0UL]) {
    iLeft = 0UL;
    frac = (u0 - bp0[0UL]) / (bp0[1UL] - bp0[0UL]);
  } else if (u0 < bp0[maxIndex]) {
    /* Binary Search */
    bpIdx = maxIndex >> 1UL;
    iLeft = 0UL;
    iRght = maxIndex;
    while (iRght - iLeft > 1UL) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = iRght + iLeft >> 1UL;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1UL] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1UL;
    frac = (u0 - bp0[maxIndex - 1UL]) / (bp0[maxIndex] - bp0[maxIndex - 1UL]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'wrapping'
   */
  return (table[iLeft + 1UL] - table[iLeft]) * frac + table[iLeft];
}

/* S-Function Block: <S14>/Function Definition */
void exported_ccp_daq_trigger(uint8_T * msg_pointer)
{
  /* Output and update for function-call system: '<S14>/Send DAQ Message' */

  /* S-Function (sfun_get_expr): '<S16>/Input Driver' */
  {
    int_T i1;
    uint8_T *y0 = &Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[0];
    for (i1=0; i1 < 8; i1++) {
      y0[i1] = (msg_pointer[i1]);
    }
  }

  /* S-Function (c280xcanxmt): '<S16>/eCAN Transmit' */
  {
    ECanaMboxes.MBOX1.MDH.byte.BYTE4 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[7];
    ECanaMboxes.MBOX1.MDH.byte.BYTE5 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[6];
    ECanaMboxes.MBOX1.MDH.byte.BYTE6 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[5];
    ECanaMboxes.MBOX1.MDH.byte.BYTE7 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[4];
    ECanaMboxes.MBOX1.MDL.byte.BYTE0 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[3];
    ECanaMboxes.MBOX1.MDL.byte.BYTE1 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[2];
    ECanaMboxes.MBOX1.MDL.byte.BYTE2 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[1];
    ECanaMboxes.MBOX1.MDL.byte.BYTE3 =
      Fuel_Pump_V3_1_20201113_20KHz_B.InputDriver[0];
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanaRegs.CANTRS.all = (((uint32_T) 0x00000001) << 1);
    EDIS;
    while (ECanaRegs.CANTA.bit.TA1 != 1 ) {
    }                              /* check eCAN Transmit Acknowledge register*/

    ECanaRegs.CANTA.bit.TA1 = 1;
                             /* clear eCAN Transmit Acknowledge register 	    */
  }
}

/* Hardware Interrupt Block: '<Root>/C28x Hardware Interrupt' */
void isr_int1pie6_task_fcn(void)
{
  /* Call the system: <Root>/Motor_Control */
  {
    /* S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */
    Fuel_Pump_V3__Motor_Control();

    /* End of Outputs for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */
  }
}

/* Hardware Interrupt Block: '<Root>/C28x Hardware Interrupt' */
void isr_int8pie1_task_fcn(void)
{
  /* Call the system: <Root>/IIC_Interrupt  */
  {
    /* S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */

    /* Output and update for function-call system: '<Root>/IIC_Interrupt ' */

    /* MATLAB Function: '<S4>/MATLAB Function' */
    switch (TEMP_Flag) {
     case 1U:
      /* Outputs for Function Call SubSystem: '<S4>/Subsystem' */
      /* S-Function (c280xi2c_rx): '<S25>/I2C Receive' */
      {
        int rx_loop= 0;
        I2caRegs.I2CSAR = 64;          /* Set slave address*/
        I2caRegs.I2CCNT= 2;            /* Set data length */

        /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
           free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
           bit count:0 (0:8bit) NACK mode:0 (1:enabled 0: disabled) stop condition:1 (1:enabled 0: disabled)*/
        I2caRegs.I2CMDR.all = 27680;
        rx_loop= 0;
        while (I2caRegs.I2CFFRX.bit.RXFFST==0 && rx_loop<10000)
          rx_loop++;
        if (rx_loop!=10000) {
          Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[0] = I2caRegs.I2CDRR;
        }

        rx_loop= 0;
        while (I2caRegs.I2CFFRX.bit.RXFFST==0 && rx_loop<10000)
          rx_loop++;
        if (rx_loop!=10000) {
          Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[1] = I2caRegs.I2CDRR;
        }
      }

      /* DataStoreWrite: '<S25>/Data Store Write2' incorporates:
       *  MATLAB Function: '<S25>/Temp_conv'
       */
      Fuel_Pump_V3_1_20_Temp_conv(Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[0],
        Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[1], &TEMP_MOT1);

      /* End of Outputs for SubSystem: '<S4>/Subsystem' */
      break;

     case 2U:
      /* Outputs for Function Call SubSystem: '<S4>/Subsystem1' */
      /* S-Function (c280xi2c_rx): '<S26>/I2C Receive' */
      {
        int rx_loop= 0;
        I2caRegs.I2CSAR = 65;          /* Set slave address*/
        I2caRegs.I2CCNT= 2;            /* Set data length */

        /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
           free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
           bit count:0 (0:8bit) NACK mode:0 (1:enabled 0: disabled) stop condition:1 (1:enabled 0: disabled)*/
        I2caRegs.I2CMDR.all = 27680;
        rx_loop= 0;
        while (I2caRegs.I2CFFRX.bit.RXFFST==0 && rx_loop<10000)
          rx_loop++;
        if (rx_loop!=10000) {
          Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[0] = I2caRegs.I2CDRR;
        }

        rx_loop= 0;
        while (I2caRegs.I2CFFRX.bit.RXFFST==0 && rx_loop<10000)
          rx_loop++;
        if (rx_loop!=10000) {
          Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[1] = I2caRegs.I2CDRR;
        }
      }

      /* DataStoreWrite: '<S26>/Data Store Write2' incorporates:
       *  MATLAB Function: '<S26>/Temp_conv'
       */
      Fuel_Pump_V3_1_20_Temp_conv(Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[0],
        Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[1], &TEMP_MOT2);

      /* End of Outputs for SubSystem: '<S4>/Subsystem1' */
      break;
    }

    /* End of MATLAB Function: '<S4>/MATLAB Function' */

    /* End of Outputs for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */
  }
}

/* Idle Task Block: '<Root>/Idle Task' */
void idle_num1_task_fcn(void)
{
  /* Call the system: <Root>/Function-Call Subsystem1 */
  {
    /* S-Function (idletask): '<Root>/Idle Task' */

    /* Output and update for function-call system: '<Root>/Function-Call Subsystem1' */
    {
      /* local block i/o variables */
      real32_T rtb_TEMP;

      /* MATLAB Function: '<S3>/READ Temp' incorporates:
       *  DataStoreRead: '<S3>/Data Store Read'
       */
      Fuel_Pump_V3_1_202_READTemp(ATEMP_MCU1, &rtb_TEMP);
      Fuel_Pump_V3_MovingAverage1(rtb_TEMP,
        &Fuel_Pump_V3_1_20201113_20KHz_B.MovingAverage,
        &Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage);

      /* DataStoreWrite: '<S3>/Data Store Write1' */
      TEMP_MCU1 = Fuel_Pump_V3_1_20201113_20KHz_B.MovingAverage.MovingAverage1;

      /* MATLAB Function: '<S3>/READ Temp1' incorporates:
       *  DataStoreRead: '<S3>/Data Store Read1'
       */
      Fuel_Pump_V3_1_202_READTemp(ATEMP_MCU2, &rtb_TEMP);
      Fuel_Pump_V3_MovingAverage1(rtb_TEMP,
        &Fuel_Pump_V3_1_20201113_20KHz_B.MovingAverage1,
        &Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage1);

      /* DataStoreWrite: '<S3>/Data Store Write' */
      TEMP_MCU2 = Fuel_Pump_V3_1_20201113_20KHz_B.MovingAverage1.MovingAverage1;

      /* S-Function (c280xcanrcv): '<S11>/eCAN Receive' */
      {
        struct ECAN_REGS ECanaShadow;
        if (ECanaRegs.CANRMP.bit.RMP0) {
          /* reenable the mailbox to receive the next message */
          EALLOW;
          ECanaShadow.CANRMP.all = 0x0;
          ECanaShadow.CANRMP.bit.RMP0 = 1;
                                    /* request clear RMP for this mailbox only*/
          ECanaRegs.CANRMP.all = ECanaShadow.CANRMP.all;
          /* 32-bit register access is reliable only                         */
          EDIS;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] =
            ECanaMboxes.MBOX0.MDL.byte.BYTE3;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1] =
            ECanaMboxes.MBOX0.MDL.byte.BYTE2;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] =
            ECanaMboxes.MBOX0.MDL.byte.BYTE1;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] =
            ECanaMboxes.MBOX0.MDL.byte.BYTE0;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4] =
            ECanaMboxes.MBOX0.MDH.byte.BYTE7;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[5] =
            ECanaMboxes.MBOX0.MDH.byte.BYTE6;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[6] =
            ECanaMboxes.MBOX0.MDH.byte.BYTE5;
          Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[7] =
            ECanaMboxes.MBOX0.MDH.byte.BYTE4;

          /* -- Call CAN RX Fcn-Call_0 -- */

          /* Output and update for function-call system: '<S11>/DTO Processing' */
          {
            boolean_T guard1 = false;

            /* Chart: '<S15>/Chart' incorporates:
             *  SubSystem: '<S15>/CCP'
             */
            /* Chart: '<S20>/CCP Stateflow ' */
            guard1 = false;
            switch (Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras) {
             case Fuel_Pump_V3_1_202_IN_Connected:
              Fuel_Pump_V3_1_20201113_20KH_DW.command_counter =
                Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1];
              if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                  CCP_GET_CCP_VERSION) {
                setMaster_Version(0,
                                  Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[
                                  2]);
                setMaster_Version(1,
                                  Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[
                                  3]);
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_GET_CCP_VERSION);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_EXCHANGE_ID) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_EXCHANGE_ID);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_SET_MTA) {
                Fuel_Pump_V3_1_20201113_set_mta();
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_SET_MTA);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_DNLOAD) {
                c_write_uint8s(&Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3],
                               Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2],
                               getMTAPtr(0));
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_DNLOAD);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_DNLOAD_6) {
                c_write_uint8s(&Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2],
                               6, getMTAPtr(0));
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_DNLOAD_6);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_SHORT_UPLOAD) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_SHORT_UPLOAD);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_UPLOAD) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_UPLOAD);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_GET_DAQ_SIZE) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_GET_DAQ_SIZE);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_SET_DAQ_PTR) {
                c_set_element_pointer
                  (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2],
                   Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3],
                   Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4]);
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_SET_DAQ_PTR);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_WRITE_DAQ) {
                Fuel_Pump_V3_1_202011_write_daq();
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_WRITE_DAQ);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_START_STOP_ALL) {
                c_start_stop_all(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2
                                 [2]);
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_START_STOP_ALL);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_START_STOP) {
                c_start_stop(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2],
                             Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3],
                             Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4],
                             Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[5],
                             1);
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_START_STOP);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_SET_S_STATUS) {
                setS_Status(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2]);
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_SET_S_STATUS);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_GET_S_STATUS) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_GET_S_STATUS);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_TEST) {
                if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                    getStation_Address(0) &&
                    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                    getStation_Address(1)) {
                  Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_TEST);
                } else {
                  setHandled(0);
                  guard1 = true;
                }
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_CONNECT) {
                if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                    getStation_Address(0) &&
                    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                    getStation_Address(1)) {
                  Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_CONNECT);
                } else {
                  setHandled(0);
                  guard1 = true;
                }
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_DISCONNECT) {
                if ((int16_T)Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                    (int16_T)Fuel_Pump__PERMANENT_DISCONNECT) {
                  Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_DISCONNECT);
                  Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
                    Fuel_Pump_V3_1__IN_Disconnected;
                  setCurrent_State(CCP_DISCONNECTED_STATE);
                  c_reset_all_DAQ_lists();
                } else if ((int16_T)
                           Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                           (int16_T)Fuel_Pump__TEMPORARY_DISCONNECT) {
                  Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_DISCONNECT);
                  guard1 = true;
                } else {
                  Fuel_Pump_V3__unhandled_command();
                }
              } else {
                Fuel_Pump_V3__unhandled_command();
              }
              break;

             case Fuel_Pump_V3_1__IN_Disconnected:
              Fuel_Pump_V3_1_20201113_20KH_DW.command_counter =
                Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1];
              if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                  CCP_CONNECT && Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2
                  [2] == getStation_Address(0) &&
                  Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                  getStation_Address(1)) {
                Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
                  Fuel_Pump_V3_IN_NO_ACTIVE_CHILD;
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_CONNECT);
                Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
                  Fuel_Pump_V3_1_202_IN_Connected;
                setCurrent_State(CCP_CONNECTED_STATE);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_TEST) {
                if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                    getStation_Address(0)) {
                  if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                      getStation_Address(1)) {
                    Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_TEST);
                  } else {
                    setHandled(0);
                  }
                } else {
                  setHandled(0);
                }
              } else {
                setHandled(0);
              }
              break;

             default:
              /* case IN_Temporarily_Disconnected: */
              Fuel_Pump_V3_1_20201113_20KH_DW.command_counter =
                Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1];
              if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] == CCP_TEST &&
                  Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                  getStation_Address(0) &&
                  Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                  getStation_Address(1)) {
                Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_TEST);
              } else if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] ==
                         CCP_CONNECT) {
                if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] ==
                    getStation_Address(0)) {
                  if (Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] ==
                      getStation_Address(1)) {
                    Fuel_Pump_V3_1_20201113__tx_dto((uint16_T)CCP_CONNECT);
                    Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
                      Fuel_Pump_V3_1_202_IN_Connected;
                    setCurrent_State(CCP_CONNECTED_STATE);
                  } else {
                    setHandled(0);
                  }
                } else {
                  setHandled(0);
                }
              } else {
                setHandled(0);
              }
              break;
            }

            if (guard1) {
              Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
                Fue_IN_Temporarily_Disconnected;
              setCurrent_State(CCP_TEMPORARILY_DISCONNECTED_STATE);
            }

            /* End of Chart: '<S20>/CCP Stateflow ' */

            /* Chart: '<S15>/Chart' incorporates:
             *  SubSystem: '<S15>/CCP(Termination)'
             */
            /* Output and update for function-call system: '<S15>/CCP(Termination)' */

            /* S-Function (sfun_ccp_termination): '<S18>/CAN Calibration Protocol (Termination)' */

            /* Outputs for sfun_ccp_termination */
            if ((getHandled()==0) && (getCurrent_State()==CCP_CONNECTED_STATE))
            {
              /* message is unhandled, but we are connected so we must output
               * the Unknown Command response */
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[0]
                = 0xFF;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[1]
                = 0x30;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[2]
                = getData(1);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[3]
                = 0;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[4]
                = 0;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[5]
                = 0;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[6]
                = 0;
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[7]
                = 0;
            } else if (getHandled()==1) {
              /* we have a valid response */
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[0]
                = getData(0);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[1]
                = getData(1);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[2]
                = getData(2);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[3]
                = getData(3);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[4]
                = getData(4);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[5]
                = getData(5);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[6]
                = getData(6);
              Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati[7]
                = getData(7);
            }

            if ((getHandled()==1) || (getCurrent_State()==CCP_CONNECTED_STATE))
            {
              /* We either handled the message and have a valid response,
               * or we have an unknown command response */

              /* Output and update for function-call system: '<S18>/CAN Transmit' */

              /* S-Function (c280xcanxmt): '<S23>/eCAN Transmit' */
              {
                ECanaMboxes.MBOX2.MDH.byte.BYTE4 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [7];
                ECanaMboxes.MBOX2.MDH.byte.BYTE5 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [6];
                ECanaMboxes.MBOX2.MDH.byte.BYTE6 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [5];
                ECanaMboxes.MBOX2.MDH.byte.BYTE7 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [4];
                ECanaMboxes.MBOX2.MDL.byte.BYTE0 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [3];
                ECanaMboxes.MBOX2.MDL.byte.BYTE1 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [2];
                ECanaMboxes.MBOX2.MDL.byte.BYTE2 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [1];
                ECanaMboxes.MBOX2.MDL.byte.BYTE3 =
                  Fuel_Pump_V3_1_20201113_20KHz_B.CANCalibrationProtocolTerminati
                  [0];
                ECanaMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
                ECanaRegs.CANTRS.all = (((uint32_T) 0x00000001) << 2);
                EDIS;
              }
            }

            /* End of Outputs for S-Function (sfun_ccp_termination): '<S18>/CAN Calibration Protocol (Termination)' */
          }
        }
      }

      /* End of Outputs for S-Function (c280xcanrcv): '<S11>/eCAN Receive' */
    }

    /* End of Outputs for S-Function (idletask): '<Root>/Idle Task' */
  }
}

/*
 * Set which subrates need to run this base step (base rate always runs).
 * This function must be called prior to calling the model step function
 * in order to "remember" which rates need to run this base step.  The
 * buffering of events allows for overlapping preemption.
 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep(boolean_T
  *eventFlags)
{
  /* Task runs when its counter is zero, computed via rtmStepTask macro */
  eventFlags[1] = ((boolean_T)rtmStepTask(Fuel_Pump_V3_1_20201113_20KH_M, 1));
  eventFlags[2] = ((boolean_T)rtmStepTask(Fuel_Pump_V3_1_20201113_20KH_M, 2));
  eventFlags[3] = ((boolean_T)rtmStepTask(Fuel_Pump_V3_1_20201113_20KH_M, 3));
}

/*
 *   This function updates active task flag for each subrate
 * and rate transition flags for tasks that exchange data.
 * The function assumes rate-monotonic multitasking scheduler.
 * The function must be called at model base rate so that
 * the generated code self-manages all its subrates and rate
 * transition flags.
 */
#pragma CODE_SECTION (rate_monotonic_scheduler, "ramfuncs")

static void rate_monotonic_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[1])++;
  if ((Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[1]) > 4) {/* Sample time: [0.005s, 0.0s] */
    Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[1] = 0;
  }

  (Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[2])++;
  if ((Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[2]) > 19) {/* Sample time: [0.02s, 0.0s] */
    Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[2] = 0;
  }

  (Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[3])++;
  if ((Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[3]) > 24) {/* Sample time: [0.025s, 0.0s] */
    Fuel_Pump_V3_1_20201113_20KH_M->Timing.TaskCounters.TID[3] = 0;
  }
}

/*
 * Output and update for action system:
 *    '<S34>/Stop'
 *    '<S34>/Error'
 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_2020111_Stop, "ramfuncs")

void Fuel_Pump_V3_1_2020111_Stop(void)
{
  /* DataStoreWrite: '<S50>/Data Store Write' incorporates:
   *  Constant: '<S50>/Constant'
   */
  Angle = 0.0F;

  /* DataStoreWrite: '<S50>/Data Store Write1' incorporates:
   *  Constant: '<S50>/Constant1'
   */
  Stop = 1U;
}

/*
 * Output and update for atomic system:
 *    '<S60>/Embedded MATLAB Function'
 *    '<S85>/Embedded MATLAB Function'
 */
#pragma CODE_SECTION (Fuel_EmbeddedMATLABFunction, "ramfuncs")

void Fuel_EmbeddedMATLABFunction(int32_T *rty_y)
{
  *rty_y = 17877801L;
}

/* Function for MATLAB Function: '<S52>/RAW_DATA2Angle ' */
static real32_T Fuel_Pump_V3_1_20201113_20K_mod(real32_T x)
{
  real32_T r;
  if (x == 0.0F) {
    r = 0.0F;
  } else {
    r = (real32_T)fmod(x, 1.0);
    if (r == 0.0F) {
      r = 0.0F;
    } else {
      if (x < 0.0F) {
        r++;
      }
    }
  }

  return r;
}

static void Fuel_Pump__SystemCore_release_c(const
  codertarget_tic2000_blocks_SP_T *obj)
{
  uint32_T PinNameLoc;
  uint32_T SPIPinsLoc;
  if (obj->isInitialized == 1L && obj->isSetupComplete) {
    PinNameLoc = MW_UNDEFINED_VALUE;
    SPIPinsLoc = MW_UNDEFINED_VALUE;
    MW_SPI_Close(obj->MW_SPI_HANDLE, SPIPinsLoc, SPIPinsLoc, SPIPinsLoc,
                 PinNameLoc);
  }
}

static void Fuel_Pump_V_SystemCore_delete_h(const
  codertarget_tic2000_blocks_SP_T *obj)
{
  Fuel_Pump__SystemCore_release_c(obj);
}

static void matlabCodegenHandle_matlabCod_e(codertarget_tic2000_blocks_SP_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    Fuel_Pump_V_SystemCore_delete_h(obj);
  }
}

/* System initialize for function-call system: '<Root>/Motor_Control' */
#pragma CODE_SECTION (Fuel_Pum_Motor_Control_Init, "ramfuncs")

void Fuel_Pum_Motor_Control_Init(void)
{
  /* SystemInitialize for IfAction SubSystem: '<S7>/Main Case ' */
  /* SystemInitialize for Atomic SubSystem: '<S34>/AD2S1210' */
  /* SystemInitialize for IfAction SubSystem: '<S36>/If Action Subsystem' */
  /* SystemInitialize for MATLAB Function: '<S52>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count = 1U;

  /* End of SystemInitialize for SubSystem: '<S36>/If Action Subsystem' */
  /* End of SystemInitialize for SubSystem: '<S34>/AD2S1210' */

  /* SystemInitialize for IfAction SubSystem: '<S34>/Position Loop Mode' */
  /* SystemInitialize for MATLAB Function: '<S47>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count_d3 = 0.0;

  /* End of SystemInitialize for SubSystem: '<S34>/Position Loop Mode' */

  /* SystemInitialize for IfAction SubSystem: '<S34>/Speed Loop Mode' */
  /* SystemInitialize for MATLAB Function: '<S49>/MATLAB Function1' */
  Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count = 0.0F;

  /* SystemInitialize for MATLAB Function: '<S49>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count_d = 0.0;

  /* End of SystemInitialize for SubSystem: '<S34>/Speed Loop Mode' */

  /* SystemInitialize for MATLAB Function: '<S41>/MATLAB Function1' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count_a = 1.0F;

  /* End of SystemInitialize for SubSystem: '<S7>/Main Case ' */
}

/* Start for function-call system: '<Root>/Motor_Control' */
#pragma CODE_SECTION (Fuel_Pu_Motor_Control_Start, "ramfuncs")

void Fuel_Pu_Motor_Control_Start(void)
{
  codertarget_tic2000_blocks_SP_T *obj;
  uint32_T SSPinNameLoc;
  uint32_T SPIPinsLoc;

  /* Start for S-Function (c280xgpio_do): '<S7>/Digital Output' incorporates:
   *  Constant: '<S7>/Constant'
   */
  EALLOW;
  GpioCtrlRegs.GPBMUX2.all &= 0xFFFFFCFF;
  GpioCtrlRegs.GPBDIR.all |= 0x100000;
  EDIS;

  /* Start for S-Function (c280xadc): '<S7>/ADC' */
  if (adcInitFlag == 0) {
    InitAdc();
    adcInitFlag = 1;
  }

  config_ADC_A (4U, 12816U, 4U, 0U, 0U);

  /* Start for IfAction SubSystem: '<S7>/Main Case ' */
  /* Start for Atomic SubSystem: '<S34>/AD2S1210' */
  /* Start for IfAction SubSystem: '<S36>/If Action Subsystem' */
  /* Start for S-Function (c280xgpio_do): '<S52>/Digital Output2' incorporates:
   *  Constant: '<S52>/Constant'
   */
  EALLOW;
  GpioCtrlRegs.GPAMUX2.all &= 0xFFFFFF3F;
  GpioCtrlRegs.GPADIR.all |= 0x80000;
  EDIS;

  /* Start for S-Function (c280xgpio_do): '<S52>/Digital Output3' incorporates:
   *  Constant: '<S52>/Constant1'
   */
  EALLOW;
  GpioCtrlRegs.GPAMUX2.all &= 0xFFFFFCFF;
  GpioCtrlRegs.GPADIR.all |= 0x100000;
  EDIS;

  /* Start for MATLABSystem: '<S52>/SPI_Master_Transfer' */
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.matlabCodegenIsDeleted = true;
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.isInitialized = 0L;
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.matlabCodegenIsDeleted = false;
  obj = &Fuel_Pump_V3_1_20201113_20KH_DW.obj;
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.isSetupComplete = false;
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.isInitialized = 1L;
  SSPinNameLoc = MW_UNDEFINED_VALUE;
  SPIPinsLoc = MW_UNDEFINED_VALUE;
  obj->MW_SPI_HANDLE = MW_SPI_Open(0UL, SPIPinsLoc, SPIPinsLoc, SPIPinsLoc,
    SSPinNameLoc, true, 0U);
  Fuel_Pump_V3_1_20201113_20KH_DW.obj.isSetupComplete = true;

  /* Start for S-Function (c280xgpio_do): '<S52>/Digital Output' incorporates:
   *  Constant: '<S52>/Constant2'
   */
  EALLOW;
  GpioCtrlRegs.GPAMUX2.all &= 0xFFFFFF3F;
  GpioCtrlRegs.GPADIR.all |= 0x80000;
  EDIS;

  /* Start for S-Function (c280xgpio_do): '<S52>/Digital Output1' incorporates:
   *  Constant: '<S52>/Constant3'
   */
  EALLOW;
  GpioCtrlRegs.GPAMUX2.all &= 0xFFFFFCFF;
  GpioCtrlRegs.GPADIR.all |= 0x100000;
  EDIS;

  /* End of Start for SubSystem: '<S36>/If Action Subsystem' */
  /* End of Start for SubSystem: '<S34>/AD2S1210' */

  /* Start for S-Function (c280xpwm): '<S34>/ePWM' */

  /*** Initialize ePWM1 modules ***/
  {
    /*-- Setup Time-Base (TB) Submodule --*/
    EPwm1Regs.TBPRD = 3750;

    /* // Time-Base Control Register
       EPwm1Regs.TBCTL.bit.CTRMODE    = 2;          // Counter Mode
       EPwm1Regs.TBCTL.bit.SYNCOSEL   = 1;          // Sync output select
       EPwm1Regs.TBCTL.bit.PRDLD      = 0;          // Shadow select
       EPwm1Regs.TBCTL.bit.PHSEN      = 1;          // Phase load enable
       EPwm1Regs.TBCTL.bit.PHSDIR     = 0;          // Phase Direction
       EPwm1Regs.TBCTL.bit.HSPCLKDIV  = 0;          // High speed time pre-scale
       EPwm1Regs.TBCTL.bit.CLKDIV     = 0;          // Timebase clock pre-scale
     */
    EPwm1Regs.TBCTL.all = (EPwm1Regs.TBCTL.all & ~0x3FBF) | 0x16;

    /* // Time-Base Phase Register
       EPwm1Regs.TBPHS.half.TBPHS     = 0;          // Phase offset register
     */
    EPwm1Regs.TBPHS.all = (EPwm1Regs.TBPHS.all & ~0xFFFF0000) | 0x0;
    EPwm1Regs.TBCTR = 0x0000;          /* Clear counter*/

    /*-- Setup Counter_Compare (CC) Submodule --*/
    /* // Counter-Compare Control Register
       EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0;  // Compare A block operating mode.
       EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0;  // Compare B block operating mode.
       EPwm1Regs.CMPCTL.bit.LOADAMODE = 0;          // Active compare A
       EPwm1Regs.CMPCTL.bit.LOADBMODE = 0;          // Active compare A
     */
    EPwm1Regs.CMPCTL.all = (EPwm1Regs.CMPCTL.all & ~0x5F) | 0x0;
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm1Regs.CMPB = 0;

    /*-- Setup Action-Qualifier (AQ) Submodule --*/
    EPwm1Regs.AQCTLA.all = 96;
    EPwm1Regs.AQCTLB.all = 96;

    /* // Action-Qualifier Software Force Register
       EPwm1Regs.AQSFRC.bit.RLDCSF    = 3;          // Reload from Shadow options
     */
    EPwm1Regs.AQSFRC.all = (EPwm1Regs.AQSFRC.all & ~0xC0) | 0xC0;

    /* // Action-Qualifier Continuous S/W Force Register Set
       EPwm1Regs.AQCSFRC.bit.CSFA     = 0;          // Continuous Software Force on output A
       EPwm1Regs.AQCSFRC.bit.CSFB     = 0;          // Continuous Software Force on output B
     */
    EPwm1Regs.AQCSFRC.all = (EPwm1Regs.AQCSFRC.all & ~0xF) | 0x0;

    /*-- Setup Dead-Band Generator (DB) Submodule --*/
    /* // Dead-Band Generator Control Register
       EPwm1Regs.DBCTL.bit.OUT_MODE   = 3;          // Dead Band Output Mode Control
       EPwm1Regs.DBCTL.bit.IN_MODE    = 0;          // Dead Band Input Select Mode Control
       EPwm1Regs.DBCTL.bit.POLSEL     = 0;          // Polarity Select Control
     */
    EPwm1Regs.DBCTL.all = (EPwm1Regs.DBCTL.all & ~0x3F) | 0x3;
    EPwm1Regs.DBRED = 150;
    EPwm1Regs.DBFED = 150;

    /*-- Setup Event-Trigger (ET) Submodule --*/
    /* // Event-Trigger Selection and Event-Trigger Pre-Scale Register
       EPwm1Regs.ETSEL.bit.SOCAEN     = 1;          // Start of conversion A Enable
       EPwm1Regs.ETSEL.bit.SOCASEL    = 1;          // Start of conversion A Select
       EPwm1Regs.ETPS.bit.SOCAPRD     = 1;          // EPWM1SOCA Period Select
       EPwm1Regs.ETSEL.bit.SOCBEN     = 0;          // Start of conversion B Enable
       EPwm1Regs.ETSEL.bit.SOCBSEL    = 1;          // Start of conversion B Select
       EPwm1Regs.ETPS.bit.SOCBPRD     = 1;          // EPWM1SOCB Period Select
       EPwm1Regs.ETSEL.bit.INTEN      = 0;          // EPWM1INTn Enable
       EPwm1Regs.ETSEL.bit.INTSEL     = 1;          // EPWM1INTn Select
       EPwm1Regs.ETPS.bit.INTPRD      = 1;          // EPWM1INTn Period Select
     */
    EPwm1Regs.ETSEL.all = (EPwm1Regs.ETSEL.all & ~0xFF0F) | 0x1901;
    EPwm1Regs.ETPS.all = (EPwm1Regs.ETPS.all & ~0x3303) | 0x1101;

    /*-- Setup PWM-Chopper (PC) Submodule --*/
    /* // PWM-Chopper Control Register
       EPwm1Regs.PCCTL.bit.CHPEN      = 0;          // PWM chopping enable
       EPwm1Regs.PCCTL.bit.CHPFREQ    = 0;          // Chopping clock frequency
       EPwm1Regs.PCCTL.bit.OSHTWTH    = 0;          // One-shot pulse width
       EPwm1Regs.PCCTL.bit.CHPDUTY    = 0;          // Chopping clock Duty cycle
     */
    EPwm1Regs.PCCTL.all = (EPwm1Regs.PCCTL.all & ~0x7FF) | 0x0;

    /*-- Set up Trip-Zone (TZ) Submodule --*/
    EALLOW;
    EPwm1Regs.TZSEL.all = 0;

    /* // Trip-Zone Control Register
       EPwm1Regs.TZCTL.bit.TZA        = 3;          // TZ1 to TZ6 Trip Action On EPWM1A
       EPwm1Regs.TZCTL.bit.TZB        = 3;          // TZ1 to TZ6 Trip Action On EPWM1B
     */
    EPwm1Regs.TZCTL.all = (EPwm1Regs.TZCTL.all & ~0xF) | 0xF;

    /* // Trip-Zone Enable Interrupt Register
       EPwm1Regs.TZEINT.bit.OST       = 0;          // Trip Zones One Shot Int Enable
       EPwm1Regs.TZEINT.bit.CBC       = 0;          // Trip Zones Cycle By Cycle Int Enable
     */
    EPwm1Regs.TZEINT.all = (EPwm1Regs.TZEINT.all & ~0x6) | 0x0;
    EDIS;
  }

  /* Start for S-Function (c280xpwm): '<S34>/ePWM1' */

  /*** Initialize ePWM2 modules ***/
  {
    /*-- Setup Time-Base (TB) Submodule --*/
    EPwm2Regs.TBPRD = 3750;

    /* // Time-Base Control Register
       EPwm2Regs.TBCTL.bit.CTRMODE    = 2;          // Counter Mode
       EPwm2Regs.TBCTL.bit.SYNCOSEL   = 1;          // Sync output select
       EPwm2Regs.TBCTL.bit.PRDLD      = 0;          // Shadow select
       EPwm2Regs.TBCTL.bit.PHSEN      = 1;          // Phase load enable
       EPwm2Regs.TBCTL.bit.PHSDIR     = 0;          // Phase Direction
       EPwm2Regs.TBCTL.bit.HSPCLKDIV  = 0;          // High speed time pre-scale
       EPwm2Regs.TBCTL.bit.CLKDIV     = 0;          // Timebase clock pre-scale
     */
    EPwm2Regs.TBCTL.all = (EPwm2Regs.TBCTL.all & ~0x3FBF) | 0x16;

    /* // Time-Base Phase Register
       EPwm2Regs.TBPHS.half.TBPHS     = 0;          // Phase offset register
     */
    EPwm2Regs.TBPHS.all = (EPwm2Regs.TBPHS.all & ~0xFFFF0000) | 0x0;
    EPwm2Regs.TBCTR = 0x0000;          /* Clear counter*/

    /*-- Setup Counter_Compare (CC) Submodule --*/
    /* // Counter-Compare Control Register
       EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;  // Compare A block operating mode.
       EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;  // Compare B block operating mode.
       EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;          // Active compare A
       EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;          // Active compare A
     */
    EPwm2Regs.CMPCTL.all = (EPwm2Regs.CMPCTL.all & ~0x5F) | 0x0;
    EPwm2Regs.CMPA.half.CMPA = 0;
    EPwm2Regs.CMPB = 0;

    /*-- Setup Action-Qualifier (AQ) Submodule --*/
    EPwm2Regs.AQCTLA.all = 96;
    EPwm2Regs.AQCTLB.all = 96;

    /* // Action-Qualifier Software Force Register
       EPwm2Regs.AQSFRC.bit.RLDCSF    = 3;          // Reload from Shadow options
     */
    EPwm2Regs.AQSFRC.all = (EPwm2Regs.AQSFRC.all & ~0xC0) | 0xC0;

    /* // Action-Qualifier Continuous S/W Force Register Set
       EPwm2Regs.AQCSFRC.bit.CSFA     = 0;          // Continuous Software Force on output A
       EPwm2Regs.AQCSFRC.bit.CSFB     = 0;          // Continuous Software Force on output B
     */
    EPwm2Regs.AQCSFRC.all = (EPwm2Regs.AQCSFRC.all & ~0xF) | 0x0;

    /*-- Setup Dead-Band Generator (DB) Submodule --*/
    /* // Dead-Band Generator Control Register
       EPwm2Regs.DBCTL.bit.OUT_MODE   = 3;          // Dead Band Output Mode Control
       EPwm2Regs.DBCTL.bit.IN_MODE    = 0;          // Dead Band Input Select Mode Control
       EPwm2Regs.DBCTL.bit.POLSEL     = 0;          // Polarity Select Control
     */
    EPwm2Regs.DBCTL.all = (EPwm2Regs.DBCTL.all & ~0x3F) | 0x3;
    EPwm2Regs.DBRED = 150;
    EPwm2Regs.DBFED = 150;

    /*-- Setup Event-Trigger (ET) Submodule --*/
    /* // Event-Trigger Selection and Event-Trigger Pre-Scale Register
       EPwm2Regs.ETSEL.bit.SOCAEN     = 0;          // Start of conversion A Enable
       EPwm2Regs.ETSEL.bit.SOCASEL    = 2;          // Start of conversion A Select
       EPwm2Regs.ETPS.bit.SOCAPRD     = 1;          // EPWM2SOCA Period Select
       EPwm2Regs.ETSEL.bit.SOCBEN     = 0;          // Start of conversion B Enable
       EPwm2Regs.ETSEL.bit.SOCBSEL    = 1;          // Start of conversion B Select
       EPwm2Regs.ETPS.bit.SOCBPRD     = 1;          // EPWM2SOCB Period Select
       EPwm2Regs.ETSEL.bit.INTEN      = 0;          // EPWM2INTn Enable
       EPwm2Regs.ETSEL.bit.INTSEL     = 1;          // EPWM2INTn Select
       EPwm2Regs.ETPS.bit.INTPRD      = 1;          // EPWM2INTn Period Select
     */
    EPwm2Regs.ETSEL.all = (EPwm2Regs.ETSEL.all & ~0xFF0F) | 0x1201;
    EPwm2Regs.ETPS.all = (EPwm2Regs.ETPS.all & ~0x3303) | 0x1101;

    /*-- Setup PWM-Chopper (PC) Submodule --*/
    /* // PWM-Chopper Control Register
       EPwm2Regs.PCCTL.bit.CHPEN      = 0;          // PWM chopping enable
       EPwm2Regs.PCCTL.bit.CHPFREQ    = 0;          // Chopping clock frequency
       EPwm2Regs.PCCTL.bit.OSHTWTH    = 0;          // One-shot pulse width
       EPwm2Regs.PCCTL.bit.CHPDUTY    = 0;          // Chopping clock Duty cycle
     */
    EPwm2Regs.PCCTL.all = (EPwm2Regs.PCCTL.all & ~0x7FF) | 0x0;

    /*-- Set up Trip-Zone (TZ) Submodule --*/
    EALLOW;
    EPwm2Regs.TZSEL.all = 0;

    /* // Trip-Zone Control Register
       EPwm2Regs.TZCTL.bit.TZA        = 3;          // TZ1 to TZ6 Trip Action On EPWM2A
       EPwm2Regs.TZCTL.bit.TZB        = 3;          // TZ1 to TZ6 Trip Action On EPWM2B
     */
    EPwm2Regs.TZCTL.all = (EPwm2Regs.TZCTL.all & ~0xF) | 0xF;

    /* // Trip-Zone Enable Interrupt Register
       EPwm2Regs.TZEINT.bit.OST       = 0;          // Trip Zones One Shot Int Enable
       EPwm2Regs.TZEINT.bit.CBC       = 0;          // Trip Zones Cycle By Cycle Int Enable
     */
    EPwm2Regs.TZEINT.all = (EPwm2Regs.TZEINT.all & ~0x6) | 0x0;
    EDIS;
  }

  /* Start for S-Function (c280xpwm): '<S34>/ePWM2' */

  /*** Initialize ePWM3 modules ***/
  {
    /*-- Setup Time-Base (TB) Submodule --*/
    EPwm3Regs.TBPRD = 3750;

    /* // Time-Base Control Register
       EPwm3Regs.TBCTL.bit.CTRMODE    = 2;          // Counter Mode
       EPwm3Regs.TBCTL.bit.SYNCOSEL   = 1;          // Sync output select
       EPwm3Regs.TBCTL.bit.PRDLD      = 0;          // Shadow select
       EPwm3Regs.TBCTL.bit.PHSEN      = 1;          // Phase load enable
       EPwm3Regs.TBCTL.bit.PHSDIR     = 0;          // Phase Direction
       EPwm3Regs.TBCTL.bit.HSPCLKDIV  = 0;          // High speed time pre-scale
       EPwm3Regs.TBCTL.bit.CLKDIV     = 0;          // Timebase clock pre-scale
     */
    EPwm3Regs.TBCTL.all = (EPwm3Regs.TBCTL.all & ~0x3FBF) | 0x16;

    /* // Time-Base Phase Register
       EPwm3Regs.TBPHS.half.TBPHS     = 0;          // Phase offset register
     */
    EPwm3Regs.TBPHS.all = (EPwm3Regs.TBPHS.all & ~0xFFFF0000) | 0x0;
    EPwm3Regs.TBCTR = 0x0000;          /* Clear counter*/

    /*-- Setup Counter_Compare (CC) Submodule --*/
    /* // Counter-Compare Control Register
       EPwm3Regs.CMPCTL.bit.SHDWAMODE = 0;  // Compare A block operating mode.
       EPwm3Regs.CMPCTL.bit.SHDWBMODE = 0;  // Compare B block operating mode.
       EPwm3Regs.CMPCTL.bit.LOADAMODE = 0;          // Active compare A
       EPwm3Regs.CMPCTL.bit.LOADBMODE = 0;          // Active compare A
     */
    EPwm3Regs.CMPCTL.all = (EPwm3Regs.CMPCTL.all & ~0x5F) | 0x0;
    EPwm3Regs.CMPA.half.CMPA = 0;
    EPwm3Regs.CMPB = 0;

    /*-- Setup Action-Qualifier (AQ) Submodule --*/
    EPwm3Regs.AQCTLA.all = 96;
    EPwm3Regs.AQCTLB.all = 96;

    /* // Action-Qualifier Software Force Register
       EPwm3Regs.AQSFRC.bit.RLDCSF    = 3;          // Reload from Shadow options
     */
    EPwm3Regs.AQSFRC.all = (EPwm3Regs.AQSFRC.all & ~0xC0) | 0xC0;

    /* // Action-Qualifier Continuous S/W Force Register Set
       EPwm3Regs.AQCSFRC.bit.CSFA     = 0;          // Continuous Software Force on output A
       EPwm3Regs.AQCSFRC.bit.CSFB     = 0;          // Continuous Software Force on output B
     */
    EPwm3Regs.AQCSFRC.all = (EPwm3Regs.AQCSFRC.all & ~0xF) | 0x0;

    /*-- Setup Dead-Band Generator (DB) Submodule --*/
    /* // Dead-Band Generator Control Register
       EPwm3Regs.DBCTL.bit.OUT_MODE   = 3;          // Dead Band Output Mode Control
       EPwm3Regs.DBCTL.bit.IN_MODE    = 0;          // Dead Band Input Select Mode Control
       EPwm3Regs.DBCTL.bit.POLSEL     = 0;          // Polarity Select Control
     */
    EPwm3Regs.DBCTL.all = (EPwm3Regs.DBCTL.all & ~0x3F) | 0x3;
    EPwm3Regs.DBRED = 150;
    EPwm3Regs.DBFED = 150;

    /*-- Setup Event-Trigger (ET) Submodule --*/
    /* // Event-Trigger Selection and Event-Trigger Pre-Scale Register
       EPwm3Regs.ETSEL.bit.SOCAEN     = 0;          // Start of conversion A Enable
       EPwm3Regs.ETSEL.bit.SOCASEL    = 2;          // Start of conversion A Select
       EPwm3Regs.ETPS.bit.SOCAPRD     = 1;          // EPWM3SOCA Period Select
       EPwm3Regs.ETSEL.bit.SOCBEN     = 0;          // Start of conversion B Enable
       EPwm3Regs.ETSEL.bit.SOCBSEL    = 1;          // Start of conversion B Select
       EPwm3Regs.ETPS.bit.SOCBPRD     = 1;          // EPWM3SOCB Period Select
       EPwm3Regs.ETSEL.bit.INTEN      = 0;          // EPWM3INTn Enable
       EPwm3Regs.ETSEL.bit.INTSEL     = 1;          // EPWM3INTn Select
       EPwm3Regs.ETPS.bit.INTPRD      = 1;          // EPWM3INTn Period Select
     */
    EPwm3Regs.ETSEL.all = (EPwm3Regs.ETSEL.all & ~0xFF0F) | 0x1201;
    EPwm3Regs.ETPS.all = (EPwm3Regs.ETPS.all & ~0x3303) | 0x1101;

    /*-- Setup PWM-Chopper (PC) Submodule --*/
    /* // PWM-Chopper Control Register
       EPwm3Regs.PCCTL.bit.CHPEN      = 0;          // PWM chopping enable
       EPwm3Regs.PCCTL.bit.CHPFREQ    = 0;          // Chopping clock frequency
       EPwm3Regs.PCCTL.bit.OSHTWTH    = 0;          // One-shot pulse width
       EPwm3Regs.PCCTL.bit.CHPDUTY    = 0;          // Chopping clock Duty cycle
     */
    EPwm3Regs.PCCTL.all = (EPwm3Regs.PCCTL.all & ~0x7FF) | 0x0;

    /*-- Set up Trip-Zone (TZ) Submodule --*/
    EALLOW;
    EPwm3Regs.TZSEL.all = 0;

    /* // Trip-Zone Control Register
       EPwm3Regs.TZCTL.bit.TZA        = 3;          // TZ1 to TZ6 Trip Action On EPWM3A
       EPwm3Regs.TZCTL.bit.TZB        = 3;          // TZ1 to TZ6 Trip Action On EPWM3B
     */
    EPwm3Regs.TZCTL.all = (EPwm3Regs.TZCTL.all & ~0xF) | 0xF;

    /* // Trip-Zone Enable Interrupt Register
       EPwm3Regs.TZEINT.bit.OST       = 0;          // Trip Zones One Shot Int Enable
       EPwm3Regs.TZEINT.bit.CBC       = 0;          // Trip Zones Cycle By Cycle Int Enable
     */
    EPwm3Regs.TZEINT.all = (EPwm3Regs.TZEINT.all & ~0x6) | 0x0;
    EDIS;
  }

  /* End of Start for SubSystem: '<S7>/Main Case ' */

  /* Start for S-Function (c280xgpio_do): '<S7>/Digital Output1' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  EALLOW;
  GpioCtrlRegs.GPBMUX2.all &= 0xFFFFFCFF;
  GpioCtrlRegs.GPBDIR.all |= 0x100000;
  EDIS;
}

/* Output and update for function-call system: '<Root>/Motor_Control' */
#pragma CODE_SECTION (Fuel_Pump_V3__Motor_Control, "ramfuncs")

void Fuel_Pump_V3__Motor_Control(void)
{
  /* local block i/o variables */
  uint16_T rtb_DataStoreRead5_p;
  uint16_T rtb_Sum1_d[3];
  int16_T VecSector;
  real32_T Angle_0;
  uint16_T rdDataRaw;
  uint16_T status;
  MW_SPI_Mode_type ClockModeValue;
  MW_SPI_FirstBitTransfer_Type MsbFirstTransferLoc;
  int32_T rtb_Sum2_a;
  int32_T rtb_y_o;
  int32_T rtb_Product_a;
  int32_T rtb_Product_ew;
  real32_T rtb_UnitDelay6;
  real32_T rtb_Gain2_n;
  real32_T rtb_Switch_i_idx_0;
  real32_T rtb_Switch_i_idx_1;
  real32_T rtb_Switch1_idx_1;
  int32_T rtb_Sum2_e_tmp;

  /* S-Function (c280xgpio_do): '<S7>/Digital Output' incorporates:
   *  Constant: '<S7>/Constant'
   */
  {
    if (1U)
      GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    else
      GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
  }

  /* S-Function (c280xadc): '<S7>/ADC' */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.ADC[0] = (AdcRegs.ADCRESULT0) >> 4;
    Fuel_Pump_V3_1_20201113_20KHz_B.ADC[1] = (AdcRegs.ADCRESULT1) >> 4;
    Fuel_Pump_V3_1_20201113_20KHz_B.ADC[2] = (AdcRegs.ADCRESULT2) >> 4;
    Fuel_Pump_V3_1_20201113_20KHz_B.ADC[3] = (AdcRegs.ADCRESULT3) >> 4;
    Fuel_Pump_V3_1_20201113_20KHz_B.ADC[4] = (AdcRegs.ADCRESULT4) >> 4;
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 0x1;/* Sequencer reset*/
  }

  /* DataStoreWrite: '<S7>/Data Store Write' */
  U1 = Fuel_Pump_V3_1_20201113_20KHz_B.ADC[0];

  /* DataStoreWrite: '<S7>/Data Store Write1' */
  V1 = Fuel_Pump_V3_1_20201113_20KHz_B.ADC[1];

  /* DataStoreWrite: '<S7>/Data Store Write2' */
  W1 = Fuel_Pump_V3_1_20201113_20KHz_B.ADC[2];

  /* DataStoreWrite: '<S7>/Data Store Write3' */
  ATEMP_MCU1 = Fuel_Pump_V3_1_20201113_20KHz_B.ADC[3];

  /* DataStoreWrite: '<S7>/Data Store Write4' */
  ATEMP_MCU2 = Fuel_Pump_V3_1_20201113_20KHz_B.ADC[4];

  /* Outputs for Atomic SubSystem: '<S7>/Interrupt_Count' */
  /* Sum: '<S33>/Add' incorporates:
   *  Constant: '<S33>/Constant'
   *  UnitDelay: '<S33>/Unit Delay'
   */
  Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_l++;

  /* Saturate: '<S33>/Saturation' incorporates:
   *  UnitDelay: '<S33>/Unit Delay'
   */
  if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_l >= 40000U) {
    Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_l = 40000U;
  }

  /* End of Saturate: '<S33>/Saturation' */

  /* SwitchCase: '<S7>/Switch Case' incorporates:
   *  Constant: '<S33>/Constant1'
   *  Product: '<S33>/Product'
   *  UnitDelay: '<S33>/Unit Delay'
   */
  switch ((int32_T)(uint16_T)(5.0E-5F * (real32_T)
           Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_l)) {
   case 0L:
    break;

   case 1L:
    /* Outputs for IfAction SubSystem: '<S7>/Offset Case ' incorporates:
     *  ActionPort: '<S35>/Action Port'
     */
    /* Sum: '<S35>/Add1' incorporates:
     *  DataStoreRead: '<S35>/Data Store Read1'
     *  Gain: '<S35>/Gain1'
     *  UnitDelay: '<S35>/Unit Delay1'
     */
    rtb_UnitDelay6 = (real32_T)(1.2566370614359172 * (real_T)V1) +
      Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay1_DSTATE;

    /* DataTypeConversion: '<S35>/Data Type Conversion4' incorporates:
     *  DataStoreWrite: '<S35>/Data Store Write1'
     */
    Offset_V1 = (uint16_T)rtb_UnitDelay6;

    /* Gain: '<S35>/Gain4' incorporates:
     *  UnitDelay: '<S35>/Unit Delay1'
     */
    Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay1_DSTATE = -0.256637067F *
      rtb_UnitDelay6;

    /* Sum: '<S35>/Add2' incorporates:
     *  DataStoreRead: '<S35>/Data Store Read2'
     *  Gain: '<S35>/Gain2'
     *  UnitDelay: '<S35>/Unit Delay2'
     */
    rtb_UnitDelay6 = (real32_T)(1.2566370614359172 * (real_T)W1) +
      Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay2_DSTATE;

    /* DataTypeConversion: '<S35>/Data Type Conversion3' incorporates:
     *  DataStoreWrite: '<S35>/Data Store Write2'
     */
    Offset_W1 = (uint16_T)rtb_UnitDelay6;

    /* Gain: '<S35>/Gain5' incorporates:
     *  UnitDelay: '<S35>/Unit Delay2'
     */
    Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay2_DSTATE = -0.256637067F *
      rtb_UnitDelay6;

    /* Sum: '<S35>/Add6' incorporates:
     *  DataStoreRead: '<S35>/Data Store Read'
     *  Gain: '<S35>/Gain12'
     *  UnitDelay: '<S35>/Unit Delay6'
     */
    rtb_UnitDelay6 = (real32_T)(1.2566370614359172 * (real_T)U1) +
      Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay6_DSTATE;

    /* DataTypeConversion: '<S35>/Data Type Conversion5' incorporates:
     *  DataStoreWrite: '<S35>/Data Store Write'
     */
    Offset_U1 = (uint16_T)rtb_UnitDelay6;

    /* Update for UnitDelay: '<S35>/Unit Delay6' incorporates:
     *  Gain: '<S35>/Gain3'
     */
    Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay6_DSTATE = -0.256637067F *
      rtb_UnitDelay6;

    /* End of Outputs for SubSystem: '<S7>/Offset Case ' */
    break;

   case 2L:
    /* Outputs for IfAction SubSystem: '<S7>/Main Case ' incorporates:
     *  ActionPort: '<S34>/Action Port'
     */
    /* Outputs for Atomic SubSystem: '<S34>/AD2S1210' */
    /* If: '<S36>/If' incorporates:
     *  DataStoreRead: '<S36>/Data Store Read1'
     */
    if (Ready_1210 > 0.0F) {
      /* Outputs for IfAction SubSystem: '<S36>/If Action Subsystem' incorporates:
       *  ActionPort: '<S52>/Action Port'
       */
      /* S-Function (c280xgpio_do): '<S52>/Digital Output2' incorporates:
       *  Constant: '<S52>/Constant'
       */
      {
        if (0.0)
          GpioDataRegs.GPASET.bit.GPIO19 = 1;
        else
          GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
      }

      /* S-Function (c280xgpio_do): '<S52>/Digital Output3' incorporates:
       *  Constant: '<S52>/Constant1'
       */
      {
        if (0.0)
          GpioDataRegs.GPASET.bit.GPIO20 = 1;
        else
          GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;
      }

      /* MATLABSystem: '<S52>/SPI_Master_Transfer' incorporates:
       *  DataStoreRead: '<S52>/Data Store Read2'
       *  DataStoreWrite: '<S52>/Data Store Write'
       */
      MW_SPI_SetSlaveSelect(Fuel_Pump_V3_1_20201113_20KH_DW.obj.MW_SPI_HANDLE,
                            0U, true);
      ClockModeValue = MW_SPI_MODE_0;
      MsbFirstTransferLoc = MW_SPI_MOST_SIGNIFICANT_BIT_FIRST;
      status = MW_SPI_SetFormat
        (Fuel_Pump_V3_1_20201113_20KH_DW.obj.MW_SPI_HANDLE, 16U, ClockModeValue,
         MsbFirstTransferLoc);
      if (status == 0U) {
        MW_SPI_MasterWriteRead_8bits
          (Fuel_Pump_V3_1_20201113_20KH_DW.obj.MW_SPI_HANDLE, &TX_DATA,
           &rdDataRaw, 1UL);
      }

      RX_DATA = rdDataRaw;

      /* End of MATLABSystem: '<S52>/SPI_Master_Transfer' */

      /* S-Function (c280xgpio_do): '<S52>/Digital Output' incorporates:
       *  Constant: '<S52>/Constant2'
       */
      {
        if (1.0)
          GpioDataRegs.GPASET.bit.GPIO19 = 1;
        else
          GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
      }

      /* S-Function (c280xgpio_do): '<S52>/Digital Output1' incorporates:
       *  Constant: '<S52>/Constant3'
       */
      {
        if (1.0)
          GpioDataRegs.GPASET.bit.GPIO20 = 1;
        else
          GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;
      }

      /* MATLAB Function: '<S52>/MATLAB Function1' incorporates:
       *  DataStoreRead: '<S52>/Data Store Read1'
       *  DataStoreWrite: '<S52>/Data Store Write'
       *  DataTypeConversion: '<S52>/Data Type Conversion'
       *  DataTypeConversion: '<S52>/Data Type Conversion1'
       *  UnitDelay: '<S52>/Unit Delay'
       */
      rtb_UnitDelay6 = fabsf((real32_T)((int32_T)RX_DATA -
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_c));
      if (rtb_UnitDelay6 < 1000.0F) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_c = RX_DATA;
      } else {
        if (rtb_UnitDelay6 > 63000.0F) {
          Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_c = RX_DATA;
        }
      }

      if (ControlMode == 0U) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_c = RX_DATA;
      }

      /* End of MATLAB Function: '<S52>/MATLAB Function1' */

      /* MATLAB Function: '<S52>/RAW_DATA2Angle ' incorporates:
       *  DataStoreRead: '<S52>/Data Store Read'
       *  DataStoreWrite: '<S52>/Data Store Write2'
       *  UnitDelay: '<S52>/Unit Delay'
       */
      Angle_0 = (real32_T)((int32_T)Mid_Position -
                           Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_c);
      if (Angle_0 < 0.0F) {
        Angle_0 += 65535.0F;
      }

      rtb_UnitDelay6 = Angle_0 * 1.5259E-5F;
      Angle_0 = Angle_0 * 4.0F * 1.5259E-5F;
      AD_Angle = Fuel_Pump_V3_1_20201113_20K_mod(Angle_0);
      rtb_UnitDelay6 = Fuel_Pump_V3_1_20201113_20K_mod(rtb_UnitDelay6);

      /* MATLAB Function: '<S52>/MATLAB Function' */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count > 20U) {
        Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count = 1U;

        /* Outputs for IfAction SubSystem: '<S52>/If Action Subsystem' incorporates:
         *  ActionPort: '<S53>/Action Port'
         */
        /* If: '<S52>/If' incorporates:
         *  Constant: '<S57>/Constant'
         *  DataStoreRead: '<S57>/Data Store Read1'
         *  DataStoreRead: '<S57>/Data Store Read2'
         *  Delay: '<S53>/Delay1'
         *  Gain: '<S57>/Gain'
         *  Gain: '<S57>/Gain1'
         *  MATLAB Function: '<S52>/RAW_DATA2Angle '
         *  MATLAB Function: '<S53>/MATLAB Function1'
         *  Product: '<S57>/Product'
         *  Product: '<S57>/Product1'
         *  Sum: '<S57>/Add'
         *  Sum: '<S57>/Add3'
         *  UnitDelay: '<S57>/Unit Delay3'
         * */
        Angle_0 = (rtb_UnitDelay6 -
                   Fuel_Pump_V3_1_20201113_20KH_DW.Delay1_DSTATE) * 2.0F *
          3.14159274F;
        if (Angle_0 > 3.1415926535897931) {
          Angle_0 -= 6.28318548F;
        } else {
          if (Angle_0 < -3.1415926535897931) {
            Angle_0 += 6.28318548F;
          }
        }

        Fuel_Pump_V3_1_20201113_20KHz_B.Add3 = Angle_0 / 0.001F * 30.0F /
          3.14159274F * (0.00628318544F * n_Filter) +
          Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay3_DSTATE;
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay3_DSTATE = (1.0F -
          0.00628318544F * n_Filter) * Fuel_Pump_V3_1_20201113_20KHz_B.Add3;
        Fuel_Pump_V3_1_20201113_20KH_DW.Delay1_DSTATE = rtb_UnitDelay6;

        /* End of If: '<S52>/If' */
        /* End of Outputs for SubSystem: '<S52>/If Action Subsystem' */
      } else {
        rdDataRaw = Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count +
          /*MW:OvSatOk*/ 1U;
        if (rdDataRaw < Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count) {
          rdDataRaw = MAX_uint16_T;
        }

        Fuel_Pump_V3_1_20201113_20KH_DW.AD2S_n_Count = rdDataRaw;
      }

      /* End of MATLAB Function: '<S52>/MATLAB Function' */

      /* DataStoreWrite: '<S52>/Data Store Write3' */
      n_1210 = Fuel_Pump_V3_1_20201113_20KHz_B.Add3;

      /* End of Outputs for SubSystem: '<S36>/If Action Subsystem' */
    }

    /* End of If: '<S36>/If' */

    /* DataStoreWrite: '<S36>/Data Store Write3' */
    n = Fuel_Pump_V3_1_20201113_20KHz_B.Add3;

    /* End of Outputs for SubSystem: '<S34>/AD2S1210' */

    /* SwitchCase: '<S34>/Switch Case' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read'
     */
    switch ((int32_T)ControlMode) {
     case 0L:
      /* Outputs for IfAction SubSystem: '<S34>/Stop' incorporates:
       *  ActionPort: '<S50>/Action Port'
       */
      Fuel_Pump_V3_1_2020111_Stop();

      /* End of Outputs for SubSystem: '<S34>/Stop' */
      break;

     case 1L:
      break;

     case 2L:
      /* Outputs for IfAction SubSystem: '<S34>/Current Loop Mode' incorporates:
       *  ActionPort: '<S39>/Action Port'
       */
      /* DataStoreRead: '<S39>/Data Store Read1' */
      rtb_y_o = IF_Frq;

      /* S-Function (tidmcrampcntl): '<S39>/Ramp Control' */

      /* C28x DMC Library (tidmcrampcntl) - '<S39>/Ramp Control' */
      {
        int32_T* ptrrampDlyCntl =
          &Fuel_Pump_V3_1_20201113_20KH_DW.RampControl_RAMP_DLY_CNTL_p;
        int32_T* ptrOldSetPoint =
          &Fuel_Pump_V3_1_20201113_20KH_DW.RampControl_PREV_SETPOINT_n;
        if (rtb_y_o != *ptrOldSetPoint ) {
          /* Original location of the update, now moved after the "if" branch */
          if (*ptrrampDlyCntl >= (long) 1000) {
            if (rtb_y_o >= *ptrOldSetPoint) {
              Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o = *ptrOldSetPoint
                + _IQ29(0.0000305);
              if (Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o > _IQ29(1.0F))
                Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o = _IQ29(1.0F);
              *ptrrampDlyCntl = 0;
            } else {
              Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o = *ptrOldSetPoint
                - _IQ29(0.0000305);
              if (Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o < _IQ29(0.0F))
                Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o = _IQ29(0.0F);
              *ptrrampDlyCntl = 0;
            }

            *ptrOldSetPoint++ = Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o;
          }

          /* Moved the update here to get more consistent Simulink time change */
          *ptrrampDlyCntl += 1;
        } else {
          Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o = *ptrOldSetPoint;
          Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o2_h = 0x7FFFFFFF;
        }

        ptrrampDlyCntl++;
      }

      /* MATLAB Function: '<S60>/Embedded MATLAB Function' */
      Fuel_EmbeddedMATLABFunction(&rtb_y_o);

      /* Sum: '<S59>/Sum2' incorporates:
       *  Product: '<S59>/Product1'
       *  S-Function (scheckfractionlength): '<S59>/ '
       *  UnitDelay: '<S59>/Unit Delay'
       */
      Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p += __IQmpy(rtb_y_o,
        Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1_o, 29);

      /* Switch: '<S61>/Switch' incorporates:
       *  Constant: '<S61>/1'
       *  RelationalOperator: '<S61>/Relational Operator'
       *  Sum: '<S61>/Sum2'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p > 536870912L) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p -= 536870912L;
      }

      /* End of Switch: '<S61>/Switch' */

      /* Switch: '<S61>/Switch1' incorporates:
       *  Constant: '<S61>/1'
       *  RelationalOperator: '<S61>/Relational Operator1'
       *  Sum: '<S61>/Sum1'
       *  UnitDelay: '<S59>/Unit Delay'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p < -536870912L) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p += 536870912L;
      }

      /* End of Switch: '<S61>/Switch1' */

      /* Switch: '<S62>/Switch2' incorporates:
       *  Constant: '<S62>/1'
       *  Product: '<S59>/Product2'
       *  RelationalOperator: '<S62>/Relational Operator2'
       *  Sum: '<S62>/Sum4'
       *  UnitDelay: '<S59>/Unit Delay'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p > 536870912L) {
        rtb_y_o = Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p -
          536870912L;
      } else {
        rtb_y_o = Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE_p;
      }

      /* End of Switch: '<S62>/Switch2' */

      /* Switch: '<S62>/Switch3' incorporates:
       *  Constant: '<S62>/1'
       *  RelationalOperator: '<S62>/Relational Operator3'
       *  Sum: '<S62>/Sum3'
       */
      if (rtb_y_o < -536870912L) {
        rtb_y_o += 536870912L;
      }

      /* End of Switch: '<S62>/Switch3' */

      /* DataTypeConversion: '<S39>/Data Type Conversion' incorporates:
       *  DataStoreWrite: '<S39>/Data Store Write'
       */
      Angle = (real32_T)rtb_y_o * 1.86264515E-9F;

      /* End of Outputs for SubSystem: '<S34>/Current Loop Mode' */
      break;

     case 3L:
      /* Outputs for IfAction SubSystem: '<S34>/Position Loop Mode' incorporates:
       *  ActionPort: '<S47>/Action Port'
       */
      /* DataStoreRead: '<S47>/Data Store Read' incorporates:
       *  DataStoreWrite: '<S47>/Data Store Write'
       */
      Angle = AD_Angle;

      /* MATLAB Function: '<S47>/MATLAB Function' */
      Fuel_Pump_V3_1_20201113_20KH_DW.count_d3++;
      if (Fuel_Pump_V3_1_20201113_20KH_DW.count_d3 >= 20.0) {
        /* Outputs for Function Call SubSystem: '<S47>/If Action Subsystem' */
        /* Switch: '<S71>/Switch1' incorporates:
         *  Constant: '<S71>/Constant2'
         *  Constant: '<S71>/Constant3'
         */
        rtb_Switch_i_idx_0 = 0.0F;
        rtb_Switch_i_idx_1 = 0.0F;

        /* DataTypeConversion: '<S73>/Data Type Conversion' incorporates:
         *  DataStoreRead: '<S73>/Data Store Read'
         *  DataTypeConversion: '<S74>/Data Type Conversion'
         */
        rtb_Product_a = (int32_T)(real32_T)floor(PK_Kpn * 65536.0F);

        /* Product: '<S73>/Product' incorporates:
         *  DataTypeConversion: '<S73>/Data Type Conversion'
         */
        rtb_Product_ew = __IQmpy(0L, rtb_Product_a, 16);

        /* DataTypeConversion: '<S75>/Data Type Conversion' incorporates:
         *  DataStoreRead: '<S75>/Data Store Read'
         *  DataTypeConversion: '<S76>/Data Type Conversion'
         */
        rtb_y_o = (int32_T)(real32_T)floor(PK_Kin * 65536.0F);

        /* DataTypeConversion: '<S73>/Data Type Conversion2' incorporates:
         *  DataStoreRead: '<S73>/Data Store Read1'
         *  DataTypeConversion: '<S74>/Data Type Conversion2'
         */
        rtb_Sum2_e_tmp = (int32_T)(real32_T)floor(uin * 65536.0F);

        /* Sum: '<S75>/Sum1' incorporates:
         *  DataTypeConversion: '<S73>/Data Type Conversion2'
         *  DataTypeConversion: '<S75>/Data Type Conversion'
         *  Product: '<S75>/Product'
         */
        rtb_Sum2_a = rtb_Sum2_e_tmp + __IQmpy(rtb_y_o, rtb_Product_ew, 16);

        /* Switch: '<S71>/Switch' incorporates:
         *  DataStoreRead: '<S71>/Data Store Read1'
         *  DataStoreRead: '<S71>/Data Store Read2'
         *  Gain: '<S71>/Gain2'
         *  Gain: '<S71>/Gain3'
         */
        if (Stop == 0U) {
          /* Saturate: '<S73>/Saturation1' */
          if (rtb_Sum2_a > 65536L) {
            rtb_Sum2_a = 65536L;
          } else {
            if (rtb_Sum2_a < -65536L) {
              rtb_Sum2_a = -65536L;
            }
          }

          /* End of Saturate: '<S73>/Saturation1' */

          /* Sum: '<S73>/Sum1' */
          rtb_Sum2_a += rtb_Product_ew;

          /* Saturate: '<S73>/Saturation' */
          if (rtb_Sum2_a > 65536L) {
            rtb_Sum2_a = 65536L;
          } else {
            if (rtb_Sum2_a < -65536L) {
              rtb_Sum2_a = -65536L;
            }
          }

          /* End of Saturate: '<S73>/Saturation' */
          rtb_Switch_i_idx_0 = (real32_T)__IQxmpy(1172812403L, rtb_Sum2_a, -12) *
            1.52587891E-5F;
          rtb_Switch_i_idx_1 = (real32_T)(int32_T)(real32_T)floor(6.66666674E-5F
            * n * 65536.0F) * 1.52587891E-5F;
        }

        /* End of Switch: '<S71>/Switch' */

        /* Sum: '<S74>/Sum2' incorporates:
         *  DataTypeConversion: '<S71>/Data Type Conversion2'
         */
        rtb_Sum2_a = (int32_T)(real32_T)floor(rtb_Switch_i_idx_0 * 65536.0F) -
          (int32_T)(real32_T)floor(rtb_Switch_i_idx_1 * 65536.0F);

        /* Product: '<S74>/Product' */
        rtb_Product_a = __IQmpy(rtb_Sum2_a, rtb_Product_a, 16);

        /* Sum: '<S76>/Sum1' incorporates:
         *  Product: '<S76>/Product'
         */
        rtb_y_o = rtb_Sum2_e_tmp + __IQmpy(rtb_y_o, rtb_Product_a, 16);

        /* Saturate: '<S74>/Saturation1' */
        if (rtb_y_o > 65536L) {
          rtb_y_o = 65536L;
        } else {
          if (rtb_y_o < -65536L) {
            rtb_y_o = -65536L;
          }
        }

        /* End of Saturate: '<S74>/Saturation1' */

        /* Saturate: '<S74>/Saturation' incorporates:
         *  Sum: '<S74>/Sum1'
         */
        un_PI = (real32_T)(rtb_Product_a + rtb_y_o) * 1.52587891E-5F;
        if (un_PI > 1.0F) {
          un_PI = 1.0F;
        } else {
          if (un_PI < -1.0F) {
            un_PI = -1.0F;
          }
        }

        /* End of Saturate: '<S74>/Saturation' */

        /* DataStoreWrite: '<S71>/Data Store Write1' incorporates:
         *  DataStoreWrite: '<S71>/Data Store Write'
         */
        IqDem = un_PI;

        /* DataStoreWrite: '<S74>/Data Store Write1' incorporates:
         *  DataTypeConversion: '<S74>/Data Type Conversion1'
         */
        uin = (real32_T)rtb_y_o * 1.52587891E-5F;

        /* DataStoreWrite: '<S74>/Data Store Write' incorporates:
         *  DataTypeConversion: '<S74>/Data Type Conversion5'
         */
        en = (real32_T)rtb_Sum2_a * 1.52587891E-5F;

        /* End of Outputs for SubSystem: '<S47>/If Action Subsystem' */
        Fuel_Pump_V3_1_20201113_20KH_DW.count_d3 = 0.0;
      }

      /* End of MATLAB Function: '<S47>/MATLAB Function' */
      /* End of Outputs for SubSystem: '<S34>/Position Loop Mode' */
      break;

     case 4L:
      /* Outputs for IfAction SubSystem: '<S34>/VVF Mode' incorporates:
       *  ActionPort: '<S51>/Action Port'
       */
      /* DataStoreRead: '<S51>/Rate' */
      rtb_y_o = VVF_Frq;

      /* S-Function (tidmcrampcntl): '<S51>/Ramp Control' */

      /* C28x DMC Library (tidmcrampcntl) - '<S51>/Ramp Control' */
      {
        int32_T* ptrrampDlyCntl =
          &Fuel_Pump_V3_1_20201113_20KH_DW.RampControl_RAMP_DLY_CNTL;
        int32_T* ptrOldSetPoint =
          &Fuel_Pump_V3_1_20201113_20KH_DW.RampControl_PREV_SETPOINT;
        if (rtb_y_o != *ptrOldSetPoint ) {
          /* Original location of the update, now moved after the "if" branch */
          if (*ptrrampDlyCntl >= (long) 1) {
            if (rtb_y_o >= *ptrOldSetPoint) {
              Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 = *ptrOldSetPoint +
                _IQ29(0.0000305);
              if (Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 > _IQ29(1.0F))
                Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 = _IQ29(1.0F);
              *ptrrampDlyCntl = 0;
            } else {
              Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 = *ptrOldSetPoint -
                _IQ29(0.0000305);
              if (Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 < _IQ29(0.0F))
                Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 = _IQ29(0.0F);
              *ptrrampDlyCntl = 0;
            }

            *ptrOldSetPoint++ = Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1;
          }

          /* Moved the update here to get more consistent Simulink time change */
          *ptrrampDlyCntl += 1;
        } else {
          Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1 = *ptrOldSetPoint;
          Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o2 = 0x7FFFFFFF;
        }

        ptrrampDlyCntl++;
      }

      /* MATLAB Function: '<S85>/Embedded MATLAB Function' */
      Fuel_EmbeddedMATLABFunction(&rtb_y_o);

      /* Sum: '<S84>/Sum2' incorporates:
       *  Product: '<S84>/Product1'
       *  S-Function (scheckfractionlength): '<S84>/ '
       *  UnitDelay: '<S84>/Unit Delay'
       */
      Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE += __IQmpy(rtb_y_o,
        Fuel_Pump_V3_1_20201113_20KHz_B.RampControl_o1, 29);

      /* Switch: '<S86>/Switch' incorporates:
       *  Constant: '<S86>/1'
       *  RelationalOperator: '<S86>/Relational Operator'
       *  Sum: '<S86>/Sum2'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE > 536870912L) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE -= 536870912L;
      }

      /* End of Switch: '<S86>/Switch' */

      /* Switch: '<S86>/Switch1' incorporates:
       *  Constant: '<S86>/1'
       *  RelationalOperator: '<S86>/Relational Operator1'
       *  Sum: '<S86>/Sum1'
       *  UnitDelay: '<S84>/Unit Delay'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE < -536870912L) {
        Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE += 536870912L;
      }

      /* End of Switch: '<S86>/Switch1' */

      /* Switch: '<S87>/Switch2' incorporates:
       *  Constant: '<S87>/1'
       *  Product: '<S84>/Product2'
       *  RelationalOperator: '<S87>/Relational Operator2'
       *  Sum: '<S87>/Sum4'
       *  UnitDelay: '<S84>/Unit Delay'
       */
      if (Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE > 536870912L) {
        rtb_y_o = Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE - 536870912L;
      } else {
        rtb_y_o = Fuel_Pump_V3_1_20201113_20KH_DW.UnitDelay_DSTATE;
      }

      /* End of Switch: '<S87>/Switch2' */

      /* Switch: '<S87>/Switch3' incorporates:
       *  Constant: '<S87>/1'
       *  RelationalOperator: '<S87>/Relational Operator3'
       *  Sum: '<S87>/Sum3'
       */
      if (rtb_y_o < -536870912L) {
        rtb_y_o += 536870912L;
      }

      /* End of Switch: '<S87>/Switch3' */

      /* DataTypeConversion: '<S51>/Data Type Conversion' incorporates:
       *  DataStoreWrite: '<S51>/Data Store Write'
       */
      Angle = (real32_T)rtb_y_o * 1.86264515E-9F;

      /* End of Outputs for SubSystem: '<S34>/VVF Mode' */
      break;

     case 5L:
      /* Outputs for IfAction SubSystem: '<S34>/Speed Loop Mode' incorporates:
       *  ActionPort: '<S49>/Action Port'
       */
      /* MATLAB Function: '<S49>/MATLAB Function1' incorporates:
       *  DataStoreRead: '<S49>/Data Store Read'
       *  DataStoreRead: '<S49>/Data Store Read1'
       *  DataStoreRead: '<S49>/Data Store Read2'
       *  DataStoreWrite: '<S49>/Data Store Write'
       */
      if (SMO_Flag == 0U) {
        Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count = 0.0F;
        Angle = AD_Angle;
      } else {
        Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count++;
        if (Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count >= 2000.0F) {
          Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count = 2000.0F;
        }

        Angle = (2000.0F - Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count) / 2000.0F *
          AD_Angle + Fuel_Pump_V3_1_20201113_20KH_DW.SMO_Count / 2000.0F *
          SMO_Theta;
      }

      /* End of MATLAB Function: '<S49>/MATLAB Function1' */

      /* MATLAB Function: '<S49>/MATLAB Function' */
      Fuel_Pump_V3_1_20201113_20KH_DW.count_d++;
      if (Fuel_Pump_V3_1_20201113_20KH_DW.count_d >= 20.0) {
        /* Outputs for Function Call SubSystem: '<S49>/If Action Subsystem' */
        /* Switch: '<S79>/Switch' incorporates:
         *  Constant: '<S79>/Constant'
         *  Constant: '<S79>/Constant1'
         *  DataStoreRead: '<S79>/Data Store Read1'
         *  DataStoreRead: '<S79>/Data Store Read2'
         *  DataStoreRead: '<S79>/Data Store Read3'
         */
        if (Stop != 0U) {
          rtb_Switch_i_idx_0 = 0.0F;
          rtb_Switch_i_idx_1 = 0.0F;
        } else {
          rtb_Switch_i_idx_0 = nDem_PI;
          rtb_Switch_i_idx_1 = n;
        }

        /* End of Switch: '<S79>/Switch' */

        /* Sum: '<S82>/Sum2' */
        rtb_UnitDelay6 = rtb_Switch_i_idx_0 - rtb_Switch_i_idx_1;

        /* Product: '<S82>/Product' incorporates:
         *  DataStoreRead: '<S82>/Data Store Read'
         */
        Angle_0 = rtb_UnitDelay6 * PK_Kpn;

        /* Sum: '<S83>/Sum3' incorporates:
         *  DataStoreRead: '<S82>/Data Store Read1'
         *  DataStoreRead: '<S83>/Data Store Read2'
         *  Gain: '<S83>/Gain'
         *  Product: '<S83>/Product1'
         */
        rtb_UnitDelay6 = 0.001F * PK_Kin * rtb_UnitDelay6 + uin;

        /* Sum: '<S82>/Sum1' */
        Angle_0 += rtb_UnitDelay6;

        /* Saturate: '<S82>/Saturation' */
        if (Angle_0 > 75.0F) {
          IqDem = 75.0F;
        } else if (Angle_0 < -75.0F) {
          IqDem = -75.0F;
        } else {
          IqDem = Angle_0;
        }

        /* End of Saturate: '<S82>/Saturation' */

        /* DataStoreWrite: '<S82>/Data Store Write' */
        uin = rtb_UnitDelay6;

        /* End of Outputs for SubSystem: '<S49>/If Action Subsystem' */
        Fuel_Pump_V3_1_20201113_20KH_DW.count_d = 0.0;
      }

      /* End of MATLAB Function: '<S49>/MATLAB Function' */
      /* End of Outputs for SubSystem: '<S34>/Speed Loop Mode' */
      break;

     default:
      /* Outputs for IfAction SubSystem: '<S34>/Error' incorporates:
       *  ActionPort: '<S42>/Action Port'
       */
      Fuel_Pump_V3_1_2020111_Stop();

      /* End of Outputs for SubSystem: '<S34>/Error' */
      break;
    }

    /* End of SwitchCase: '<S34>/Switch Case' */

    /* DataStoreRead: '<S34>/Data Store Read5' */
    rtb_DataStoreRead5_p = Stop;

    /* If: '<S34>/If' */
    if (rtb_DataStoreRead5_p == 1U) {
      /* Outputs for IfAction SubSystem: '<S34>/If Action Subsystem' incorporates:
       *  ActionPort: '<S43>/Action Port'
       */
      /* DataStoreWrite: '<S43>/Data Store Write1' incorporates:
       *  Constant: '<S43>/Constant'
       */
      uiId = 0.0F;

      /* DataStoreWrite: '<S43>/Data Store Write2' incorporates:
       *  Constant: '<S43>/Constant1'
       */
      uiIq = 0.0F;

      /* DataStoreWrite: '<S43>/Data Store Write3' incorporates:
       *  Constant: '<S43>/Constant2'
       */
      uin = 0.0F;

      /* End of Outputs for SubSystem: '<S34>/If Action Subsystem' */
    }

    /* End of If: '<S34>/If' */

    /* Gain: '<S34>/Gain' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read1'
     *  DataStoreRead: '<S34>/Data Store Read3'
     *  DataStoreWrite: '<S34>/Data Store Write6'
     *  Sum: '<S34>/Sum'
     */
    Current_U1 = (real32_T)((int16_T)U1 - (int16_T)Offset_U1) * 0.0732421875F;

    /* Gain: '<S34>/Gain1' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read6'
     *  DataStoreRead: '<S34>/Data Store Read8'
     *  DataStoreWrite: '<S34>/Data Store Write8'
     *  Sum: '<S34>/Sum1'
     */
    Current_V1 = (real32_T)((int16_T)V1 - (int16_T)Offset_V1) * 0.0732421875F;

    /* Gain: '<S34>/Gain2' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read4'
     *  DataStoreRead: '<S34>/Data Store Read7'
     *  DataStoreWrite: '<S34>/Data Store Write9'
     *  Sum: '<S34>/Sum2'
     */
    Current_W1 = (real32_T)((int16_T)W1 - (int16_T)Offset_W1) * 0.0732421875F;

    /* MATLAB Function: '<S34>/MATLAB Function' incorporates:
     *  DataStoreWrite: '<S34>/Data Store Write6'
     *  DataStoreWrite: '<S34>/Data Store Write8'
     *  DataStoreWrite: '<S34>/Data Store Write9'
     */
    rtb_UnitDelay6 = fabsf(Current_U1);
    if (rtb_UnitDelay6 > 60.0F || fabsf(Current_V1) > 60.0F || fabsf(Current_W1)
        > 60.0F) {
      rdDataRaw = RMS_Count + /*MW:OvSatOk*/ 1U;
      if (rdDataRaw < RMS_Count) {
        rdDataRaw = MAX_uint16_T;
      }

      RMS_Count = rdDataRaw;
      if (RMS_Count >= 5U) {
        Protect_Flag = 2U;
      }
    } else {
      RMS_Count = 0U;
    }

    if (rtb_UnitDelay6 > 85.0F) {
      Protect_Flag = 1U;
    } else if (fabsf(Current_V1) > 85.0F) {
      Protect_Flag = 1U;
    } else {
      if (fabsf(Current_W1) > 85.0F) {
        Protect_Flag = 1U;
      }
    }

    if (Protect_Flag != 0U) {
      Stop = 1U;
    }

    /* End of MATLAB Function: '<S34>/MATLAB Function' */

    /* Gain: '<S38>/Gain2' incorporates:
     *  DataStoreWrite: '<S34>/Data Store Write6'
     *  DataStoreWrite: '<S34>/Data Store Write8'
     *  DataStoreWrite: '<S34>/Data Store Write9'
     *  Gain: '<S38>/Gain'
     *  Gain: '<S38>/Gain1'
     *  Sum: '<S38>/Add'
     */
    rtb_Gain2_n = (Current_U1 - 0.5F * Current_V1 - 0.5F * Current_W1) *
      0.666666687F;

    /* Gain: '<S46>/Gain' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read15'
     */
    rtb_Switch_i_idx_1 = 6.28318548F * Angle;

    /* Trigonometry: '<S46>/Sin1' */
    rtb_Switch_i_idx_0 = (real32_T)cos(rtb_Switch_i_idx_1);

    /* Gain: '<S38>/Gain3' incorporates:
     *  DataStoreWrite: '<S34>/Data Store Write8'
     *  DataStoreWrite: '<S34>/Data Store Write9'
     *  Sum: '<S38>/Add1'
     */
    rtb_UnitDelay6 = (Current_V1 - Current_W1) * 0.577350259F;

    /* Trigonometry: '<S46>/Sin' */
    rtb_Switch_i_idx_1 = (real32_T)sin(rtb_Switch_i_idx_1);

    /* Sum: '<S46>/Add' incorporates:
     *  Product: '<S46>/Product'
     *  Product: '<S46>/Product1'
     */
    Angle_0 = rtb_Gain2_n * rtb_Switch_i_idx_0 + rtb_UnitDelay6 *
      rtb_Switch_i_idx_1;

    /* Sum: '<S46>/Add1' incorporates:
     *  Product: '<S46>/Product2'
     *  Product: '<S46>/Product3'
     */
    rtb_Gain2_n = rtb_UnitDelay6 * rtb_Switch_i_idx_0 - rtb_Gain2_n *
      rtb_Switch_i_idx_1;

    /* Outputs for Atomic SubSystem: '<S34>/Current_PI' */
    /* Switch: '<S40>/Switch' incorporates:
     *  Constant: '<S40>/Constant'
     *  Constant: '<S40>/Constant1'
     *  DataStoreRead: '<S40>/Data Store Read1'
     *  DataStoreRead: '<S40>/Data Store Read2'
     *  DataStoreRead: '<S40>/Data Store Read4'
     *  Switch: '<S40>/Switch1'
     */
    if (Stop != 0U) {
      rtb_Switch_i_idx_0 = 0.0F;
      rtb_Switch_i_idx_1 = 0.0F;
      rtb_UnitDelay6 = 0.0F;
      rtb_Switch1_idx_1 = 0.0F;
    } else {
      rtb_Switch_i_idx_0 = IdDem;
      rtb_Switch_i_idx_1 = Angle_0;
      rtb_UnitDelay6 = IqDem;
      rtb_Switch1_idx_1 = rtb_Gain2_n;
    }

    /* End of Switch: '<S40>/Switch' */

    /* Sum: '<S67>/Sum2' */
    rtb_UnitDelay6 -= rtb_Switch1_idx_1;

    /* Product: '<S67>/Product' incorporates:
     *  DataStoreRead: '<S67>/Data Store Read'
     */
    rtb_Switch1_idx_1 = rtb_UnitDelay6 * PK_KpIq;

    /* Sum: '<S69>/Sum3' incorporates:
     *  DataStoreRead: '<S67>/Data Store Read1'
     *  DataStoreRead: '<S69>/Data Store Read2'
     *  Gain: '<S69>/Gain'
     *  Product: '<S69>/Product1'
     */
    rtb_UnitDelay6 = 5.0E-5F * PK_KiIq * rtb_UnitDelay6 + uiIq;

    /* Sum: '<S67>/Sum1' */
    Uq = rtb_Switch1_idx_1 + rtb_UnitDelay6;

    /* Saturate: '<S67>/Saturation' */
    if (Uq > 28.0F) {
      /* Sum: '<S67>/Sum1' */
      Uq = 28.0F;
    } else {
      if (Uq < -28.0F) {
        /* Sum: '<S67>/Sum1' */
        Uq = -28.0F;
      }
    }

    /* End of Saturate: '<S67>/Saturation' */

    /* Sum: '<S66>/Sum2' */
    rtb_Switch_i_idx_0 -= rtb_Switch_i_idx_1;

    /* Product: '<S66>/Product' incorporates:
     *  DataStoreRead: '<S66>/Data Store Read'
     */
    rtb_Switch_i_idx_1 = rtb_Switch_i_idx_0 * PK_KpId;

    /* Sum: '<S68>/Sum3' incorporates:
     *  DataStoreRead: '<S66>/Data Store Read1'
     *  DataStoreRead: '<S68>/Data Store Read2'
     *  Gain: '<S68>/Gain'
     *  Product: '<S68>/Product1'
     */
    rtb_Switch_i_idx_0 = 5.0E-5F * PK_KiId * rtb_Switch_i_idx_0 + uiId;

    /* Sum: '<S66>/Sum1' */
    Ud = rtb_Switch_i_idx_1 + rtb_Switch_i_idx_0;

    /* Saturate: '<S66>/Saturation' */
    if (Ud > 28.0F) {
      /* Sum: '<S66>/Sum1' */
      Ud = 28.0F;
    } else {
      if (Ud < -28.0F) {
        /* Sum: '<S66>/Sum1' */
        Ud = -28.0F;
      }
    }

    /* End of Saturate: '<S66>/Saturation' */

    /* MATLAB Function: '<S40>/MATLAB Function1' */
    Us = (real32_T)sqrt(Ud * Ud + Uq * Uq);
    if (Us > 28.0F) {
      /* Sum: '<S66>/Sum1' */
      Ud /= Us;

      /* Sum: '<S67>/Sum1' */
      Uq /= Us;
    }

    if (ControlMode == 4U) {
      /* Sum: '<S66>/Sum1' */
      Ud = Ud_VVF;

      /* Sum: '<S67>/Sum1' */
      Uq = Uq_VVF;
    }

    /* End of MATLAB Function: '<S40>/MATLAB Function1' */

    /* DataStoreWrite: '<S66>/Data Store Write' */
    uiId = rtb_Switch_i_idx_0;

    /* DataStoreWrite: '<S67>/Data Store Write' */
    uiIq = rtb_UnitDelay6;

    /* DataStoreWrite: '<S40>/Data Store Write2' */
    Id = Angle_0;

    /* DataStoreWrite: '<S40>/Data Store Write3' */
    Iq = rtb_Gain2_n;

    /* End of Outputs for SubSystem: '<S34>/Current_PI' */

    /* Outputs for Atomic SubSystem: '<S34>/SVPWM' */
    /* Gain: '<S77>/Gain' incorporates:
     *  DataStoreRead: '<S34>/Data Store Read2'
     */
    rtb_UnitDelay6 = 6.28318548F * Angle;

    /* Trigonometry: '<S77>/Sin1' */
    rtb_Switch_i_idx_0 = (real32_T)cos(rtb_UnitDelay6);

    /* Trigonometry: '<S77>/Sin' */
    rtb_UnitDelay6 = (real32_T)sin(rtb_UnitDelay6);

    /* Sum: '<S77>/Sum2' incorporates:
     *  DataStoreWrite: '<S48>/Data Store Write'
     *  Product: '<S77>/Product'
     *  Product: '<S77>/Product1'
     */
    V_alpha = Ud * rtb_Switch_i_idx_0 - Uq * rtb_UnitDelay6;

    /* Sum: '<S77>/Sum1' incorporates:
     *  DataStoreWrite: '<S48>/Data Store Write1'
     *  Product: '<S77>/Product2'
     *  Product: '<S77>/Product3'
     */
    V_beta = Uq * rtb_Switch_i_idx_0 + Ud * rtb_UnitDelay6;

    /* MATLAB Function: '<S48>/MATLAB Function' incorporates:
     *  DataStoreWrite: '<S48>/Data Store Write'
     *  DataStoreWrite: '<S48>/Data Store Write1'
     */
    Angle_0 = 0.5F * V_beta + 0.86603F * V_alpha;
    rtb_Gain2_n = Angle_0 - V_beta;
    VecSector = 3;
    if (Angle_0 > 0.0F) {
      VecSector = 2;
    }

    if (rtb_Gain2_n > 0.0F) {
      VecSector--;
    }

    if (V_beta < 0.0F) {
      VecSector = 7 - VecSector;
    }

    switch (VecSector) {
     case 1:
      rtb_UnitDelay6 = Angle_0;
      rtb_Gain2_n = V_beta - rtb_Gain2_n;
      Angle_0 = -Angle_0;
      break;

     case 2:
      rtb_UnitDelay6 = rtb_Gain2_n + Angle_0;
      rtb_Gain2_n = V_beta;
      Angle_0 = -V_beta;
      break;

     case 3:
      rtb_UnitDelay6 = rtb_Gain2_n;
      rtb_Gain2_n = -rtb_Gain2_n;
      Angle_0 = -(V_beta + Angle_0);
      break;

     case 4:
      rtb_UnitDelay6 = Angle_0;
      rtb_Gain2_n = V_beta - rtb_Gain2_n;
      Angle_0 = -Angle_0;
      break;

     case 5:
      rtb_UnitDelay6 = rtb_Gain2_n + Angle_0;
      rtb_Gain2_n = V_beta;
      Angle_0 = -V_beta;
      break;

     default:
      rtb_UnitDelay6 = rtb_Gain2_n;
      rtb_Gain2_n = -rtb_Gain2_n;
      Angle_0 = -(V_beta + Angle_0);
      break;
    }

    /* End of MATLAB Function: '<S48>/MATLAB Function' */

    /* Sum: '<S48>/Sum1' incorporates:
     *  Constant: '<S48>/Constant'
     *  Gain: '<S48>/Gain'
     *  Gain: '<S48>/Gain1'
     */
    rtb_Sum1_d[0] = (uint16_T)(1875.0F - 0.0357142873F * rtb_UnitDelay6 *
      1875.0F);
    rtb_Sum1_d[1] = (uint16_T)(1875.0F - 0.0357142873F * rtb_Gain2_n * 1875.0F);
    rtb_Sum1_d[2] = (uint16_T)(1875.0F - 0.0357142873F * Angle_0 * 1875.0F);

    /* End of Outputs for SubSystem: '<S34>/SVPWM' */

    /* S-Function (c280xpwm): '<S34>/ePWM' */

    /*-- Update CMPA value for ePWM1 --*/
    {
      EPwm1Regs.CMPA.half.CMPA = (uint16_T)(rtb_Sum1_d[0]);
    }

    EPwm1Regs.AQCSFRC.bit.CSFA = rtb_DataStoreRead5_p;
    EPwm1Regs.AQCSFRC.bit.CSFB = rtb_DataStoreRead5_p;

    /* S-Function (c280xpwm): '<S34>/ePWM1' */

    /*-- Update CMPA value for ePWM2 --*/
    {
      EPwm2Regs.CMPA.half.CMPA = (uint16_T)(rtb_Sum1_d[1]);
    }

    EPwm2Regs.AQCSFRC.bit.CSFA = rtb_DataStoreRead5_p;
    EPwm2Regs.AQCSFRC.bit.CSFB = rtb_DataStoreRead5_p;

    /* S-Function (c280xpwm): '<S34>/ePWM2' */

    /*-- Update CMPA value for ePWM3 --*/
    {
      EPwm3Regs.CMPA.half.CMPA = (uint16_T)(rtb_Sum1_d[2]);
    }

    EPwm3Regs.AQCSFRC.bit.CSFA = rtb_DataStoreRead5_p;
    EPwm3Regs.AQCSFRC.bit.CSFB = rtb_DataStoreRead5_p;

    /* MATLAB Function: '<S41>/MATLAB Function1' incorporates:
     *  DataStoreRead: '<S41>/Data Store Read'
     *  DataStoreRead: '<S41>/Data Store Read1'
     *  DataStoreRead: '<S41>/Data Store Read2'
     *  DataStoreRead: '<S41>/Data Store Read3'
     *  DataStoreRead: '<S41>/Data Store Read4'
     *  DataStoreRead: '<S41>/Data Store Read5'
     */
    if (Buffer_Flag == 1U) {
      Fuel_Pump_V3_1_20201113_20KH_DW.count_a = 1.0F;
    } else if (Fuel_Pump_V3_1_20201113_20KH_DW.count_a <= 1000.0F) {
      VecSector = (int16_T)Fuel_Pump_V3_1_20201113_20KH_DW.count_a - 1;
      Data_Buffer_ThetaSMO[VecSector] = SMO_Theta;
      Data_Buffer_ThetaAD2S[VecSector] = AD_Angle;
      Data_Buffer_IqDem[VecSector] = IqDem;
      Data_Buffer_IdDem[VecSector] = IdDem;
      Data_Buffer_Iq[VecSector] = Iq;
      Data_Buffer_Id[VecSector] = Id;
      Fuel_Pump_V3_1_20201113_20KH_DW.count_a++;
    } else {
      Fuel_Pump_V3_1_20201113_20KH_DW.count_a = 1.0F;
      Buffer_Flag = 1U;
    }

    /* End of MATLAB Function: '<S41>/MATLAB Function1' */

    /* MATLAB Function: '<S34>/MATLAB Function3' */
    if (rtb_DataStoreRead5_p == 1U) {
      uin = 0.0F;
    }

    /* End of MATLAB Function: '<S34>/MATLAB Function3' */
    /* End of Outputs for SubSystem: '<S7>/Main Case ' */
    break;
  }

  /* End of SwitchCase: '<S7>/Switch Case' */
  /* End of Outputs for SubSystem: '<S7>/Interrupt_Count' */

  /* S-Function (c280xgpio_do): '<S7>/Digital Output1' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  {
    if (0U)
      GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    else
      GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
  }
}

/* Termination for function-call system: '<Root>/Motor_Control' */
#pragma CODE_SECTION (Fuel_Pum_Motor_Control_Term, "ramfuncs")

void Fuel_Pum_Motor_Control_Term(void)
{
  /* Terminate for IfAction SubSystem: '<S7>/Main Case ' */
  /* Terminate for Atomic SubSystem: '<S34>/AD2S1210' */
  /* Terminate for IfAction SubSystem: '<S36>/If Action Subsystem' */
  /* Terminate for MATLABSystem: '<S52>/SPI_Master_Transfer' */
  matlabCodegenHandle_matlabCod_e(&Fuel_Pump_V3_1_20201113_20KH_DW.obj);

  /* End of Terminate for SubSystem: '<S36>/If Action Subsystem' */
  /* End of Terminate for SubSystem: '<S34>/AD2S1210' */
  /* End of Terminate for SubSystem: '<S7>/Main Case ' */
}

/*
 * Output and update for atomic system:
 *    '<S25>/Temp_conv'
 *    '<S26>/Temp_conv'
 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20_Temp_conv, "ramfuncs")

void Fuel_Pump_V3_1_20_Temp_conv(uint16_T rtu_u1, uint16_T rtu_u2, real32_T
  *rty_y)
{
  *rty_y = (real32_T)((rtu_u1 << 8U) + rtu_u2) * 0.01284F - 260.0F;
}

static void Fuel_Pump_V3_SystemCore_release(dsp_simulink_MovingAverage_Fu_T *obj)
{
  e_dsp_private_SlidingWindowAv_T *obj_0;
  if (obj->isInitialized == 1L && obj->isSetupComplete) {
    obj_0 = obj->pStatistic;
    if (obj_0->isInitialized == 1L) {
      obj_0->isInitialized = 2L;
    }

    obj->NumChannels = -1L;
  }
}

static void Fuel_Pump_V3__SystemCore_delete(dsp_simulink_MovingAverage_Fu_T *obj)
{
  Fuel_Pump_V3_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCodeg(dsp_simulink_MovingAverage_Fu_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    Fuel_Pump_V3__SystemCore_delete(obj);
  }
}

/*
 * System initialize for atomic system:
 *    synthesized block
 *    synthesized block
 */
#pragma CODE_SECTION (Fuel_Pu_MovingAverage1_Init, "ramfuncs")

void Fuel_Pu_MovingAverage1_Init(DW_MovingAverage1_Fuel_Pump_V_T *localDW)
{
  /* InitializeConditions for MATLABSystem: '<S3>/Moving Average1' */
  if (localDW->obj.pStatistic->isInitialized == 1L) {
    localDW->obj.pStatistic->pCumSum = 0.0F;
    memset(&localDW->obj.pStatistic->pCumSumRev[0], 0, 39U * sizeof(real32_T));
    localDW->obj.pStatistic->pCumRevIndex = 1.0F;
  }

  /* End of InitializeConditions for MATLABSystem: '<S3>/Moving Average1' */
}

/*
 * System reset for atomic system:
 *    synthesized block
 *    synthesized block
 */
#pragma CODE_SECTION (Fuel_P_MovingAverage1_Reset, "ramfuncs")

void Fuel_P_MovingAverage1_Reset(DW_MovingAverage1_Fuel_Pump_V_T *localDW)
{
  /* InitializeConditions for MATLABSystem: '<S3>/Moving Average1' */
  if (localDW->obj.pStatistic->isInitialized == 1L) {
    localDW->obj.pStatistic->pCumSum = 0.0F;
    memset(&localDW->obj.pStatistic->pCumSumRev[0], 0, 39U * sizeof(real32_T));
    localDW->obj.pStatistic->pCumRevIndex = 1.0F;
  }

  /* End of InitializeConditions for MATLABSystem: '<S3>/Moving Average1' */
}

/*
 * Start for atomic system:
 *    synthesized block
 *    synthesized block
 */
#pragma CODE_SECTION (Fuel_P_MovingAverage1_Start, "ramfuncs")

void Fuel_P_MovingAverage1_Start(DW_MovingAverage1_Fuel_Pump_V_T *localDW)
{
  /* Start for MATLABSystem: '<S3>/Moving Average1' */
  localDW->obj.matlabCodegenIsDeleted = false;
  localDW->objisempty = true;
  localDW->obj.isInitialized = 1L;
  localDW->obj.NumChannels = 1L;
  localDW->gobj_0.isInitialized = 0L;
  localDW->obj.pStatistic = &localDW->gobj_0;
  localDW->obj.isSetupComplete = true;
  localDW->obj.TunablePropsChanged = false;
}

/*
 * Output and update for atomic system:
 *    synthesized block
 *    synthesized block
 */
#pragma CODE_SECTION (Fuel_Pump_V3_MovingAverage1, "ramfuncs")

void Fuel_Pump_V3_MovingAverage1(real32_T rtu_0, B_MovingAverage1_Fuel_Pump_V3_T
  *localB, DW_MovingAverage1_Fuel_Pump_V_T *localDW)
{
  real32_T cumRevIndex;
  real32_T csum;
  real32_T csumrev[39];
  real32_T z;
  int16_T z_tmp;

  /* MATLABSystem: '<S3>/Moving Average1' */
  if (localDW->obj.TunablePropsChanged) {
    localDW->obj.TunablePropsChanged = false;
  }

  if (localDW->obj.pStatistic->isInitialized != 1L) {
    localDW->obj.pStatistic->isSetupComplete = false;
    localDW->obj.pStatistic->isInitialized = 1L;
    localDW->obj.pStatistic->pCumSum = 0.0F;
    localDW->obj.pStatistic->pCumRevIndex = 1.0F;
    localDW->obj.pStatistic->isSetupComplete = true;
    localDW->obj.pStatistic->pCumSum = 0.0F;
    memset(&localDW->obj.pStatistic->pCumSumRev[0], 0, 39U * sizeof(real32_T));
    localDW->obj.pStatistic->pCumRevIndex = 1.0F;
  }

  cumRevIndex = localDW->obj.pStatistic->pCumRevIndex;
  csum = localDW->obj.pStatistic->pCumSum;
  for (z_tmp = 0; z_tmp < 39; z_tmp++) {
    csumrev[z_tmp] = localDW->obj.pStatistic->pCumSumRev[z_tmp];
  }

  csum += rtu_0;
  z_tmp = (int16_T)cumRevIndex - 1;
  z = csumrev[z_tmp] + csum;
  csumrev[z_tmp] = rtu_0;
  if (cumRevIndex != 39.0F) {
    cumRevIndex++;
  } else {
    cumRevIndex = 1.0F;
    csum = 0.0F;
    for (z_tmp = 37; z_tmp >= 0; z_tmp--) {
      csumrev[z_tmp] += csumrev[z_tmp + 1];
    }
  }

  localDW->obj.pStatistic->pCumSum = csum;
  memcpy(&localDW->obj.pStatistic->pCumSumRev[0], &csumrev[0], 39U * sizeof
         (real32_T));
  localDW->obj.pStatistic->pCumRevIndex = cumRevIndex;
  localB->MovingAverage1 = z / 40.0F;

  /* End of MATLABSystem: '<S3>/Moving Average1' */
}

/*
 * Termination for atomic system:
 *    synthesized block
 *    synthesized block
 */
#pragma CODE_SECTION (Fuel_Pu_MovingAverage1_Term, "ramfuncs")

void Fuel_Pu_MovingAverage1_Term(DW_MovingAverage1_Fuel_Pump_V_T *localDW)
{
  /* Terminate for MATLABSystem: '<S3>/Moving Average1' */
  matlabCodegenHandle_matlabCodeg(&localDW->obj);
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3_1_20201113_20_init(void)
{
  setSlave_Version(0, 2);
  setSlave_Version(1, 1);
  setStation_Address(0, 1U);
  setStation_Address(1, 0U);
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3_1_2020111_get_mta0(uint16_T first_byte)
{
  c_getUINT32bytes(getMTAPtr(0), getDataPtr(first_byte));
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3_1_20201113__tx_dto(uint16_T tx_comm_type)
{
  uint32_T memptr;
  memptr = 0UL;
  setHandled(1);
  setData(0, CCP_DTO_ID);
  setData(1, CCP_CRC_OK);
  setData(2, Fuel_Pump_V3_1_20201113_20KH_DW.command_counter);
  if (tx_comm_type == CCP_TEST || tx_comm_type == CCP_DISCONNECT || tx_comm_type
      == CCP_CONNECT || tx_comm_type == CCP_SET_MTA || tx_comm_type ==
      CCP_SET_DAQ_PTR || tx_comm_type == CCP_WRITE_DAQ || tx_comm_type ==
      CCP_START_STOP || tx_comm_type == CCP_START_STOP_ALL || tx_comm_type ==
      CCP_SET_S_STATUS) {
    setData(3, 0);
    setData(4, 0);
    setData(5, 0);
    setData(6, 0);
    setData(7, 0);
  } else if (tx_comm_type == CCP_GET_CCP_VERSION) {
    setData(3, getSlave_Version(0));
    setData(4, getSlave_Version(1));
  } else if (tx_comm_type == CCP_EXCHANGE_ID) {
    setData(3, Fuel_P_SLAVE_ID_LENGTH_NOT_USED);
    setData(4, Fue_SLAVE_ID_DATA_TYPE_NOT_USED);
    setData(5, Fuel_RESOURCE_AVAILABILITY_MASK);
    setData(6, Fuel_P_RESOURCE_PROTECTION_MASK);
  } else if (tx_comm_type == CCP_DNLOAD || tx_comm_type == CCP_DNLOAD_6) {
    setData(3, Fuel_Pump_MTA_ADDRESS_EXTENSION);
    Fuel_Pump_V3_1_2020111_get_mta0(4U);
  } else if (tx_comm_type == CCP_SHORT_UPLOAD) {
    c_setUINT32bytes(&memptr, &Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4]);
    c_read_uint8s(getDataPtr(3), Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2
                  [2], &memptr);
  } else if (tx_comm_type == CCP_UPLOAD) {
    c_read_uint8s(getDataPtr(3), Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2
                  [2], getMTAPtr(0));
  } else if (tx_comm_type == CCP_GET_DAQ_SIZE) {
    c_init_daq_list(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2]);
    setData(5, 0);
    setData(6, 0);
    setData(7, 0);
  } else {
    if (tx_comm_type == CCP_GET_S_STATUS) {
      setData(3, getS_Status());
      setData(4, Fuel_Pump_V3_S_STATUS_QUALIFIER);
    }
  }
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3__unhandled_command(void)
{
  setHandled(0);
  setData(0, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0]);
  setData(1, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1]);
  setData(2, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2]);
  setData(3, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3]);
  setData(4, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4]);
  setData(5, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[5]);
  setData(6, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[6]);
  setData(7, Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[7]);
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3_1_202011_write_daq(void)
{
  uint32_T b_address;
  b_address = 0UL;
  c_setUINT32bytes(&b_address, &Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4]);
  c_write_daq(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2], b_address);
}

/* Function for Chart: '<S20>/CCP Stateflow ' */
static void Fuel_Pump_V3_1_20201113_set_mta(void)
{
  c_setUINT32bytes(getMTAPtr(Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2]),
                   &Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4]);
}

/*
 * Output and update for atomic system:
 *    '<S3>/READ Temp'
 *    '<S3>/READ Temp1'
 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_202_READTemp, "ramfuncs")

void Fuel_Pump_V3_1_202_READTemp(uint16_T rtu_AD, real32_T *rty_TEMP)
{
  *rty_TEMP = 1.0F / (((real32_T)log(rtu_AD) - (real32_T)log(4096.0F - (real32_T)
    rtu_AD)) * 0.00025316F + 0.00335402F) - 273.15F;
}

real32_T rt_roundf(real32_T u)
{
  real32_T y;
  if (fabsf(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = 0.0F;
    } else {
      y = (real32_T)ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function1'
 *    '<S91>/MATLAB Function10'
 */
#pragma CODE_SECTION (Fuel_Pump_V_MATLABFunction1, "ramfuncs")

void Fuel_Pump_V_MATLABFunction1(real32_T rtu_RAW_DATA, uint16_T *rty_TX_DATA)
{
  if (-rtu_RAW_DATA >= 0.0F && -rtu_RAW_DATA <= 20000.0F) {
    *rty_TX_DATA = (uint16_T)rt_roundf(-rtu_RAW_DATA);
  } else if (-rtu_RAW_DATA > 20000.0F) {
    *rty_TX_DATA = 45536U;
  } else {
    *rty_TX_DATA = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function11'
 *    '<S91>/MATLAB Function12'
 *    '<S91>/MATLAB Function13'
 */
#pragma CODE_SECTION (Fuel_Pump__MATLABFunction11, "ramfuncs")

void Fuel_Pump__MATLABFunction11(real32_T rtu_RAW_DATA, uint16_T *rty_TX_DATA)
{
  real32_T Real_DATA;
  Real_DATA = rtu_RAW_DATA / 0.000488296151F;
  if (Real_DATA >= 0.0F && Real_DATA <= 32767.0F) {
    *rty_TX_DATA = (uint16_T)rt_roundf(Real_DATA);
  } else if (Real_DATA > 32767.0F) {
    *rty_TX_DATA = 32767U;
  } else {
    *rty_TX_DATA = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function14'
 *    '<S91>/MATLAB Function15'
 *    '<S91>/MATLAB Function16'
 *    '<S91>/MATLAB Function17'
 */
#pragma CODE_SECTION (Fuel_Pump__MATLABFunction14, "ramfuncs")

void Fuel_Pump__MATLABFunction14(real32_T rtu_RAW_DATA, uint16_T *rty_TX_DATA)
{
  real32_T Real_Current;
  uint16_T q0;
  uint16_T qY;
  Real_Current = rtu_RAW_DATA / 0.01F;
  if (Real_Current >= 0.0F && Real_Current <= 25600.0F) {
    *rty_TX_DATA = (uint16_T)rt_roundf(Real_Current);
  } else if (Real_Current > 25600.0F) {
    *rty_TX_DATA = 25600U;
  } else if (Real_Current < 0.0F && Real_Current >= -25600.0F) {
    q0 = ~(uint16_T)rt_roundf(Real_Current / -1.0F);
    qY = q0 + /*MW:OvSatOk*/ 1U;
    if (qY < q0) {
      qY = MAX_uint16_T;
    }

    *rty_TX_DATA = qY;
  } else {
    *rty_TX_DATA = 39936U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function19'
 *    '<S91>/MATLAB Function20'
 */
#pragma CODE_SECTION (Fuel_Pump__MATLABFunction19, "ramfuncs")

void Fuel_Pump__MATLABFunction19(uint16_T rtu_RAW_DATA, uint16_T *rty_TX_DATA)
{
  real32_T Real_DATA;
  Real_DATA = (real32_T)rtu_RAW_DATA / 65535.0F * 6.28318548F / 0.000191753454F;
  if (Real_DATA <= 32767.0F) {
    *rty_TX_DATA = (uint16_T)rt_roundf(Real_DATA);
  } else {
    *rty_TX_DATA = 32767U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function2'
 *    '<S91>/MATLAB Function5'
 *    '<S91>/MATLAB Function6'
 *    '<S91>/MATLAB Function7'
 *    '<S91>/MATLAB Function8'
 *    '<S91>/MATLAB Function9'
 */
#pragma CODE_SECTION (Fuel_Pump_V_MATLABFunction2, "ramfuncs")

void Fuel_Pump_V_MATLABFunction2(real32_T rtu_RAW_DATA, uint16_T *rty_TX_DATA)
{
  real32_T Real_Current;
  uint16_T q0;
  uint16_T qY;
  Real_Current = rtu_RAW_DATA * 100.0F / 0.01F;
  if (Real_Current >= 0.0F && Real_Current <= 25600.0F) {
    *rty_TX_DATA = (uint16_T)rt_roundf(Real_Current);
  } else if (Real_Current > 25600.0F) {
    *rty_TX_DATA = 25600U;
  } else if (Real_Current < 0.0F && Real_Current >= -25600.0F) {
    q0 = ~(uint16_T)rt_roundf(Real_Current / -1.0F);
    qY = q0 + /*MW:OvSatOk*/ 1U;
    if (qY < q0) {
      qY = MAX_uint16_T;
    }

    *rty_TX_DATA = qY;
  } else {
    *rty_TX_DATA = 39936U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S91>/MATLAB Function23'
 *    '<S91>/MATLAB Function24'
 *    '<S91>/MATLAB Function25'
 *    '<S91>/MATLAB Function26'
 *    '<S91>/MATLAB Function27'
 *    '<S91>/MATLAB Function28'
 *    '<S91>/MATLAB Function29'
 *    '<S91>/MATLAB Function30'
 *    '<S91>/MATLAB Function31'
 *    '<S91>/MATLAB Function32'
 *    ...
 */
#pragma CODE_SECTION (Fuel_Pump__MATLABFunction23, "ramfuncs")

void Fuel_Pump__MATLABFunction23(uint16_T rtu_TX_DATA, uint16_T *rty_TX_DATA_Hig,
  uint16_T *rty_TX_DATA_Low)
{
  *rty_TX_DATA_Hig = rtu_TX_DATA & 255U;
  *rty_TX_DATA_Low = rtu_TX_DATA >> 8U;
}

/* Output and update for atomic system: '<Root>/Set_nDem' */
#pragma CODE_SECTION (Fuel_Pump_V3_1_202_Set_nDem, "ramfuncs")

void Fuel_Pump_V3_1_202_Set_nDem(void)
{
  real32_T rtb_Gain_mc;

  /* Gain: '<S9>/Gain' incorporates:
   *  DataStoreRead: '<S9>/Data Store Read2'
   */
  rtb_Gain_mc = 8.33333324E-5F * nDem;

  /* MATLAB Function: '<S9>/MATLAB Function1' incorporates:
   *  DataStoreRead: '<S9>/Data Store Read1'
   */
  if (fabsf(rtb_Gain_mc) <= 0.25F) {
    if (rtb_Gain_mc >= SetpointValue) {
      SetpointValue += n_Step;
    } else {
      SetpointValue -= n_Step;
    }
  } else if (rtb_Gain_mc >= SetpointValue) {
    SetpointValue += n_Step / 2.0F;
  } else {
    SetpointValue -= n_Step / 2.0F;
  }

  if (fabsf(rtb_Gain_mc - SetpointValue) <= n_Step * 2.0F) {
    SetpointValue = rtb_Gain_mc;
  }

  if (SetpointValue >= 0.0F) {
    SetpointValue = 0.0F;
  } else {
    if (SetpointValue <= -1.0F) {
      SetpointValue = -1.0F;
    }
  }

  /* Gain: '<S9>/Gain1' incorporates:
   *  DataStoreWrite: '<S9>/Data Store Write1'
   *  MATLAB Function: '<S9>/MATLAB Function1'
   */
  nDem_PI = 12000.0F * SetpointValue;

  /* Gain: '<S9>/Gain6' incorporates:
   *  DataStoreWrite: '<S9>/Data Store Write1'
   *  DataStoreWrite: '<S9>/Data Store Write5'
   *  Gain: '<S9>/Gain7'
   */
  SMO_Filter = 10.0F * -nDem_PI;
}

real_T rt_roundd(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function for TID0 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step0, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_step0(void) /* Sample time: [0.001s, 0.0s] */
{
  {                                    /* Sample time: [0.001s, 0.0s] */
    rate_monotonic_scheduler();
  }

  /* S-Function (c280xgpio_di): '<S2>/Digital Input' */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[0] =
      GpioDataRegs.GPBDAT.bit.GPIO53;
    Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[1] =
      GpioDataRegs.GPBDAT.bit.GPIO54;
    Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[2] =
      GpioDataRegs.GPBDAT.bit.GPIO55;
  }

  /* DataStoreWrite: '<S2>/Data Store Write10' */
  MCU_ST = Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[0];

  /* DataStoreWrite: '<S2>/Data Store Write7' */
  GF_MODE = Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[1];

  /* DataStoreWrite: '<S2>/Data Store Write8' */
  NC1 = Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput[2];

  /* S-Function (c280xgpio_di): '<S2>/Digital Input1' */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput1 =
      GpioDataRegs.GPBDAT.bit.GPIO56;
  }

  /* DataStoreWrite: '<S2>/Data Store Write9' */
  NC2 = Fuel_Pump_V3_1_20201113_20KHz_B.DigitalInput1;

  /* Outputs for Atomic SubSystem: '<Root>/Set_nDem' */
  Fuel_Pump_V3_1_202_Set_nDem();

  /* End of Outputs for SubSystem: '<Root>/Set_nDem' */
}

/* Model step function for TID1 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step1, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_step1(void) /* Sample time: [0.005s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/CCP_Buffer' */
  /* MATLAB Function: '<S1>/MATLAB Function' */
  if (Buffer_Flag == 0U) {
    Buffer_Out_ThetaSMO = -5.0F;
    Buffer_Out_ThetaAD2S = -5.0F;
    Buffer_Out_IqDem = -5.0F;
    Buffer_Out_Iq = -5.0F;
    Buffer_Out_IdDem = -5.0F;
    Buffer_Out_Id = -5.0F;
  } else if (Fuel_Pump_V3_1_20201113_20KH_DW.count_out <= 1000.0F) {
    switch ((int16_T)Fuel_Pump_V3_1_20201113_20KH_DW.switch_count) {
     case 1:
      Buffer_Out_ThetaSMO = Data_Buffer_ThetaSMO[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 2.0F;
      break;

     case 2:
      Buffer_Out_ThetaAD2S = Data_Buffer_ThetaAD2S[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 3.0F;
      break;

     case 3:
      Buffer_Out_IqDem = Data_Buffer_IqDem[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 4.0F;
      break;

     case 4:
      Buffer_Out_Iq = Data_Buffer_Iq[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 5.0F;
      break;

     case 5:
      Buffer_Out_IdDem = Data_Buffer_IdDem[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 6.0F;
      break;

     case 6:
      Buffer_Out_Id = Data_Buffer_Id[(int16_T)
        Fuel_Pump_V3_1_20201113_20KH_DW.count_out - 1];
      Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 1.0F;
      Fuel_Pump_V3_1_20201113_20KH_DW.count_out++;
      break;
    }
  } else {
    Buffer_Out_ThetaSMO = -5.0F;
    Buffer_Out_ThetaAD2S = -5.0F;
    Buffer_Out_IqDem = -5.0F;
    Buffer_Out_Iq = -5.0F;
    Buffer_Out_IdDem = -5.0F;
    Buffer_Out_Id = -5.0F;
    Fuel_Pump_V3_1_20201113_20KH_DW.count_out = 1.0F;
    Buffer_Flag = 0U;
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function' */
  /* End of Outputs for SubSystem: '<Root>/CCP_Buffer' */
}

/* Model step function for TID2 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step2, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_step2(void) /* Sample time: [0.02s, 0.0s] */
{
  /* Outputs for Atomic SubSystem: '<Root>/IIC_PT1000' */
  /* MATLAB Function: '<S5>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count++;
  if (Fuel_Pump_V3_1_20201113_20KH_DW.count >= 3.0) {
    Fuel_Pump_V3_1_20201113_20KH_DW.count = 1.0;
  }

  switch ((int16_T)Fuel_Pump_V3_1_20201113_20KH_DW.count) {
   case 1:
    TEMP_Flag = 1U;

    /* Outputs for Function Call SubSystem: '<S5>/Subsystem4' */
    /* S-Function (c280xi2c_tx): '<S30>/I2C Transmit' incorporates:
     *  Constant: '<S30>/Constant'
     */
    {
      int unsigned tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CSAR = 64;          /* Set slave address*/
        I2caRegs.I2CCNT= 1;            /* Set data length */

        /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
           free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
           bit count:0 (0:8bit) stop condition:0 (1:enabled 0: disabled)*/
        I2caRegs.I2CMDR.all = 26144;
        tx_loop= 0;
        while (I2caRegs.I2CFFTX.bit.TXFFST==16 && tx_loop<10000)
          tx_loop++;
        if (tx_loop!=10000) {
          I2caRegs.I2CDXR = (uint8_T)16U;
        }
      }
    }

    /* End of Outputs for SubSystem: '<S5>/Subsystem4' */
    break;

   case 2:
    TEMP_Flag = 2U;

    /* Outputs for Function Call SubSystem: '<S5>/Subsystem5' */
    /* S-Function (c280xi2c_tx): '<S31>/I2C Transmit' incorporates:
     *  Constant: '<S31>/Constant'
     */
    {
      int unsigned tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CSAR = 65;          /* Set slave address*/
        I2caRegs.I2CCNT= 1;            /* Set data length */

        /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
           free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
           bit count:0 (0:8bit) stop condition:0 (1:enabled 0: disabled)*/
        I2caRegs.I2CMDR.all = 26144;
        tx_loop= 0;
        while (I2caRegs.I2CFFTX.bit.TXFFST==16 && tx_loop<10000)
          tx_loop++;
        if (tx_loop!=10000) {
          I2caRegs.I2CDXR = (uint8_T)16U;
        }
      }
    }

    /* End of Outputs for SubSystem: '<S5>/Subsystem5' */
    break;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */
  /* End of Outputs for SubSystem: '<Root>/IIC_PT1000' */
}

/* Model step function for TID3 */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step3, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_step3(void) /* Sample time: [0.025s, 0.0s] */
{
  real32_T Real_Current;
  uint16_T SCI_Buffer;
  uint32_T x;
  real_T y;
  uint16_T x_0[47];
  int16_T k;
  uint16_T rtb_TX_DATA_Hig_gp;
  uint16_T rtb_TX_DATA_Low_mw;
  uint16_T rtb_TX_DATA_Low_pr;
  uint16_T rtb_TX_DATA_Hig_dl;
  uint16_T rtb_TX_DATA_Low_h;
  uint16_T rtb_TX_DATA_Hig_i3;
  uint16_T rtb_TX_DATA_Low_bd;
  uint16_T rtb_TX_DATA_Hig_nj;
  uint16_T rtb_TX_DATA_Low_cy;
  uint16_T rtb_TX_DATA_Hig_dt;
  uint16_T rtb_TX_DATA_Low_my;
  uint16_T rtb_TX_DATA_Hig_k;
  uint16_T rtb_TX_DATA_Low_e;
  uint16_T rtb_TX_DATA_Hig_ik;
  uint16_T rtb_TX_DATA_Low_am;
  uint16_T rtb_TX_DATA_Hig_ex;
  uint16_T rtb_TX_DATA_Low_p;
  uint16_T rtb_TX_DATA_Hig_b;
  uint16_T rtb_TX_DATA_Low_ja;
  uint16_T rtb_TX_DATA_Hig_o;
  uint16_T rtb_TX_DATA_Low_jy;
  uint16_T rtb_TX_DATA_Hig_be;
  uint16_T rtb_TX_DATA_Low_pk;
  uint16_T rtb_TX_DATA_Hig_p;
  uint16_T rtb_TX_DATA_Low_c;
  uint16_T rtb_TX_DATA_Hig_fc;
  uint16_T rtb_TX_DATA_Low_ke;
  uint16_T rtb_TX_DATA_Hig_f5;
  uint16_T rtb_TX_DATA_Low_k;
  uint16_T rtb_TX_DATA_Hig_f;
  uint16_T rtb_TX_DATA_Low_m;
  uint16_T rtb_TX_DATA_Hig_ba;
  uint16_T rtb_TX_DATA_Low_k5;
  uint16_T rtb_TX_DATA_Hig_d;
  uint16_T rtb_TX_DATA_Low_by;
  uint16_T rtb_TX_DATA_Hig_nh;
  uint16_T rtb_TX_DATA_Low_nq;
  uint16_T rtb_TX_DATA_Low_cp;
  uint16_T rtb_TX_DATA_Hig_ge;
  uint16_T rtb_TX_DATA_Low_d;
  uint16_T rtb_TX_DATA_Hig;
  uint16_T rtb_TX_DATA_Low;
  uint16_T rtb_TX_DATA_Hig_oh;
  uint16_T rtb_TX_DATA_Low_eg;
  uint16_T rtb_TX_DATA_g;
  uint32_T qY;
  uint64_T y_0;

  /* MATLAB Function: '<S91>/MATLAB Function3' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read2'
   */
  Real_Current = IdDem * 100.0F / 0.01F;
  if (Real_Current >= 0.0F && Real_Current <= 25600.0F) {
    SCI_Buffer = (uint16_T)rt_roundf(Real_Current);
  } else if (Real_Current > 25600.0F) {
    SCI_Buffer = 25600U;
  } else if (Real_Current < 0.0F && Real_Current >= -25600.0F) {
    rtb_TX_DATA_Hig_gp = ~(uint16_T)rt_roundf(Real_Current / -1.0F);
    SCI_Buffer = rtb_TX_DATA_Hig_gp + /*MW:OvSatOk*/ 1U;
    if (SCI_Buffer < rtb_TX_DATA_Hig_gp) {
      SCI_Buffer = MAX_uint16_T;
    }
  } else {
    SCI_Buffer = 39936U;
  }

  /* MATLAB Function: '<S91>/MATLAB Function4' incorporates:
   *  MATLAB Function: '<S91>/MATLAB Function3'
   */
  Fuel_Pump__MATLABFunction23(SCI_Buffer, &rtb_TX_DATA_Hig_gp,
    &rtb_TX_DATA_Low_mw);

  /* MATLAB Function: '<S91>/MATLAB Function9' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read3'
   */
  Fuel_Pump_V_MATLABFunction2(Id, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function23' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &SCI_Buffer, &rtb_TX_DATA_Low_pr);

  /* MATLAB Function: '<S91>/MATLAB Function2' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read'
   */
  Fuel_Pump_V_MATLABFunction2(IqDem, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function24' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_dl,
    &rtb_TX_DATA_Low_h);

  /* MATLAB Function: '<S91>/MATLAB Function8' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read1'
   */
  Fuel_Pump_V_MATLABFunction2(Iq, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function25' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_i3,
    &rtb_TX_DATA_Low_bd);

  /* MATLAB Function: '<S91>/MATLAB Function1' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read18'
   */
  Fuel_Pump_V_MATLABFunction1(nDem_PI, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function26' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_nj,
    &rtb_TX_DATA_Low_cy);

  /* MATLAB Function: '<S91>/MATLAB Function10' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read19'
   */
  Fuel_Pump_V_MATLABFunction1(n, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function27' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_dt,
    &rtb_TX_DATA_Low_my);

  /* MATLAB Function: '<S91>/MATLAB Function11' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read14'
   */
  Fuel_Pump__MATLABFunction11(LWFDem, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function28' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_k,
    &rtb_TX_DATA_Low_e);

  /* MATLAB Function: '<S91>/MATLAB Function12' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read15'
   */
  Fuel_Pump__MATLABFunction11(LWFf, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function29' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_ik,
    &rtb_TX_DATA_Low_am);

  /* MATLAB Function: '<S91>/MATLAB Function13' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read16'
   */
  Fuel_Pump__MATLABFunction11(eLWFf, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function30' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_ex,
    &rtb_TX_DATA_Low_p);

  /* MATLAB Function: '<S91>/MATLAB Function18' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read12'
   *  Lookup_n-D: '<S91>/1-D Lookup Table1'
   */
  Real_Current = look1_iflf_binlx(n,
    Fuel_Pump_V3_1_20201113__ConstP.uDLookupTable1_bp01Data,
    Fuel_Pump_V3_1_20201113__ConstP.uDLookupTable1_tableData, 10UL) /
    0.0152592547F;
  if (Real_Current >= 0.0F && Real_Current <= 32767.0F) {
    rtb_TX_DATA_Hig_o = (uint16_T)rt_roundf(Real_Current);
  } else if (Real_Current > 32767.0F) {
    rtb_TX_DATA_Hig_o = 32767U;
  } else {
    rtb_TX_DATA_Hig_o = 0U;
  }

  /* End of MATLAB Function: '<S91>/MATLAB Function18' */

  /* MATLAB Function: '<S91>/MATLAB Function31' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_Hig_o, &rtb_TX_DATA_Hig_b,
    &rtb_TX_DATA_Low_ja);

  /* MATLAB Function: '<S91>/MATLAB Function16' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read10'
   */
  Fuel_Pump__MATLABFunction14(TEMP_MOT1, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function32' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_o,
    &rtb_TX_DATA_Low_jy);

  /* MATLAB Function: '<S91>/MATLAB Function17' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read11'
   */
  Fuel_Pump__MATLABFunction14(TEMP_MOT2, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function33' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_be,
    &rtb_TX_DATA_Low_pk);

  /* MATLAB Function: '<S91>/MATLAB Function5' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read5'
   */
  Fuel_Pump_V_MATLABFunction2(Current_U1, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function34' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_p,
    &rtb_TX_DATA_Low_c);

  /* MATLAB Function: '<S91>/MATLAB Function6' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read6'
   */
  Fuel_Pump_V_MATLABFunction2(Current_V1, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function35' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_fc,
    &rtb_TX_DATA_Low_ke);

  /* MATLAB Function: '<S91>/MATLAB Function7' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read7'
   */
  Fuel_Pump_V_MATLABFunction2(Current_W1, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function36' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_f5,
    &rtb_TX_DATA_Low_k);

  /* MATLAB Function: '<S91>/MATLAB Function14' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read8'
   */
  Fuel_Pump__MATLABFunction14(TEMP_MCU1, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function37' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_f,
    &rtb_TX_DATA_Low_m);

  /* MATLAB Function: '<S91>/MATLAB Function15' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read9'
   */
  Fuel_Pump__MATLABFunction14(TEMP_MCU2, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function38' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_ba,
    &rtb_TX_DATA_Low_k5);

  /* MATLAB Function: '<S91>/MATLAB Function19' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read17'
   */
  Fuel_Pump__MATLABFunction19(RX_DATA, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function39' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_d,
    &rtb_TX_DATA_Low_by);

  /* MATLAB Function: '<S91>/MATLAB Function20' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read20'
   */
  Fuel_Pump__MATLABFunction19(Mid_Position, &rtb_TX_DATA_g);

  /* MATLAB Function: '<S91>/MATLAB Function40' */
  Fuel_Pump__MATLABFunction23(rtb_TX_DATA_g, &rtb_TX_DATA_Hig_nh,
    &rtb_TX_DATA_Low_nq);

  /* MATLAB Function: '<S91>/MATLAB Function' */
  qY = CPU_Timer + /*MW:OvSatOk*/ 1UL;
  if (qY < CPU_Timer) {
    qY = MAX_uint32_T;
  }

  CPU_Timer = qY;
  qY = CPU_Timer / 65535UL;
  x = CPU_Timer - (uint32_T)(qY * 65535ULL);
  if (x > 0UL && x >= 32768UL) {
    qY++;
  }

  /* MATLAB Function: '<S91>/MATLAB Function41' incorporates:
   *  MATLAB Function: '<S91>/MATLAB Function'
   */
  Fuel_Pump__MATLABFunction23((uint16_T)CPU_Timer, &rtb_TX_DATA_g,
    &rtb_TX_DATA_Low_cp);

  /* MATLAB Function: '<S91>/MATLAB Function42' incorporates:
   *  MATLAB Function: '<S91>/MATLAB Function'
   */
  Fuel_Pump__MATLABFunction23((uint16_T)qY, &rtb_TX_DATA_Hig_ge,
    &rtb_TX_DATA_Low_d);

  /* MATLAB Function: '<S90>/MATLAB Function' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read22'
   *  DataStoreWrite: '<S90>/Data Store Write'
   */
  if (LWFf >= -0.6 && LWFf <= 0.6) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_c = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_c++;
  }

  bFlt_LWFf = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt_c >= 3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function' */

  /* MATLAB Function: '<S90>/MATLAB Function1' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read'
   *  DataStoreWrite: '<S90>/Data Store Write1'
   */
  if (n >= -12000.0F && n <= 5.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_k = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_k++;
  }

  bFlt_Resolver = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt_k >= 3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function1' */

  /* MATLAB Function: '<S90>/MATLAB Function2' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read1'
   *  DataStoreRead: '<S90>/Data Store Read2'
   *  DataStoreWrite: '<S90>/Data Store Write2'
   */
  if (TEMP_MCU1 <= 180.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_mo = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_mo++;
  }

  if (TEMP_MCU2 <= 180.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt2_m = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt2_m++;
  }

  bFlt_MOST_T = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_mo >= 3.0 ||
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt2_m >= 3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function2' */

  /* MATLAB Function: '<S90>/MATLAB Function3' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read3'
   *  DataStoreWrite: '<S90>/Data Store Write3'
   */
  if (TEMP_MOT1 >= -180.0F && TEMP_MOT1 <= 180.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_h = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_h++;
  }

  bFlt_WindingUT_Sensor = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt_h >=
    3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function3' */

  /* MATLAB Function: '<S90>/MATLAB Function4' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read4'
   *  DataStoreWrite: '<S90>/Data Store Write4'
   */
  if (TEMP_MOT2 >= -180.0F && TEMP_MOT2 <= 180.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt++;
  }

  bFlt_WindingVT_Sensor = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt >= 3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function4' */

  /* MATLAB Function: '<S90>/MATLAB Function5' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read5'
   *  DataStoreRead: '<S90>/Data Store Read6'
   *  DataStoreRead: '<S90>/Data Store Read7'
   *  DataStoreWrite: '<S90>/Data Store Write5'
   */
  if (Current_U1 >= -1.5F && Current_U1 <= 1.5F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_UI = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_UI++;
  }

  if (Current_V1 >= -1.5F && Current_V1 <= 1.5F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_VI = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_VI++;
  }

  if (Current_W1 >= -1.5F && Current_W1 <= 1.5F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_WI = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_WI++;
  }

  bFlt_Winding_I = (uint16_T)(Fuel_Pump_V3_1_20201113_20KH_DW.cnt_UI >= 3.0 ||
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_VI >= 3.0 ||
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt_WI >= 3.0);

  /* End of MATLAB Function: '<S90>/MATLAB Function5' */

  /* MATLAB Function: '<S90>/MATLAB Function6' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read8'
   *  DataStoreWrite: '<S90>/Data Store Write6'
   */
  bFlt_OvrLim_I = (uint16_T)(Protect_Flag != 0U);

  /* MATLAB Function: '<S90>/MATLAB Function7' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read10'
   *  DataStoreRead: '<S90>/Data Store Read9'
   *  DataStoreWrite: '<S90>/Data Store Write7'
   */
  if (TEMP_MCU1 <= 90.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_m = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_m++;
  }

  if (TEMP_MCU2 <= 120.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt2 = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt2++;
  }

  if (Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_m >= 12.0 ||
      Fuel_Pump_V3_1_20201113_20KH_DW.cnt2 >= 12.0) {
    bFlt_OvrLim_TempMOST = 1U;
  }

  bFlt_OvrLim_MOST = bFlt_OvrLim_TempMOST;

  /* End of MATLAB Function: '<S90>/MATLAB Function7' */

  /* MATLAB Function: '<S90>/MATLAB Function8' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read12'
   *  DataStoreWrite: '<S90>/Data Store Write8'
   */
  if (TEMP_MOT1 <= 120.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_k = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_k++;
  }

  if (Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_k >= 12.0) {
    bFlt_OvrLim_TempUT = 1U;
  }

  bFlt_OvrLim_WindingUT = bFlt_OvrLim_TempUT;

  /* End of MATLAB Function: '<S90>/MATLAB Function8' */

  /* MATLAB Function: '<S90>/MATLAB Function9' incorporates:
   *  DataStoreRead: '<S90>/Data Store Read13'
   *  DataStoreWrite: '<S90>/Data Store Write9'
   */
  if (TEMP_MOT2 <= 120.0F) {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1 = 0.0;
  } else {
    Fuel_Pump_V3_1_20201113_20KH_DW.cnt1++;
  }

  if (Fuel_Pump_V3_1_20201113_20KH_DW.cnt1 >= 12.0) {
    bFlt_OvrLim_TempVT = 1U;
  }

  bFlt_OvrLim_WindingVT = bFlt_OvrLim_TempVT;

  /* End of MATLAB Function: '<S90>/MATLAB Function9' */

  /* MATLAB Function: '<S91>/MATLAB Function44' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read21'
   *  DataStoreRead: '<S91>/Data Store Read22'
   *  DataStoreRead: '<S91>/Data Store Read23'
   *  DataStoreRead: '<S91>/Data Store Read24'
   *  DataStoreRead: '<S91>/Data Store Read25'
   *  DataStoreRead: '<S91>/Data Store Read26'
   *  DataStoreRead: '<S91>/Data Store Read27'
   *  DataStoreRead: '<S91>/Data Store Read28'
   *  DataStoreRead: '<S91>/Data Store Read29'
   *  DataStoreRead: '<S91>/Data Store Read30'
   *  DataStoreRead: '<S91>/Data Store Read31'
   *  MATLAB Function: '<S91>/MATLAB Function21'
   */
  Fuel_Pump__MATLABFunction23((1U == SCI_Error) + ((uint16_T)(1U == bFlt_LWFf) <<
    1U) + ((uint16_T)(1U == bFlt_Resolver) << 2U) + ((uint16_T)(1U ==
    bFlt_MOST_T) << 3U) + ((uint16_T)(1U == bFlt_WindingUT_Sensor) << 4U) +
    ((uint16_T)(1U == bFlt_WindingVT_Sensor) << 5U) + ((uint16_T)(1U ==
    bFlt_Winding_I) << 6U) + ((uint16_T)(1U == bFlt_OvrLim_I) << 7U) +
    ((uint16_T)(1U == bFlt_OvrLim_MOST) << 8U) + ((uint16_T)(1U ==
    bFlt_OvrLim_WindingUT) << 9U) + ((uint16_T)(1U == bFlt_OvrLim_WindingVT) <<
    10U), &rtb_TX_DATA_Hig, &rtb_TX_DATA_Low);

  /* MATLAB Function: '<S91>/MATLAB Function43' incorporates:
   *  DataStoreRead: '<S91>/Data Store Read4'
   */
  Fuel_Pump__MATLABFunction23(ControlMode, &rtb_TX_DATA_Hig_oh,
    &rtb_TX_DATA_Low_eg);

  /* MATLAB Function: '<S91>/MATLAB Function22' incorporates:
   *  Constant: '<S91>/Constant4'
   *  SignalConversion generated from: '<S117>/ SFunction '
   */
  x_0[0] = 18U;
  x_0[1] = rtb_TX_DATA_Hig_gp;
  x_0[2] = rtb_TX_DATA_Low_mw;
  x_0[3] = SCI_Buffer;
  x_0[4] = rtb_TX_DATA_Low_pr;
  x_0[5] = rtb_TX_DATA_Hig_dl;
  x_0[6] = rtb_TX_DATA_Low_h;
  x_0[7] = rtb_TX_DATA_Hig_i3;
  x_0[8] = rtb_TX_DATA_Low_bd;
  x_0[9] = rtb_TX_DATA_Hig_nj;
  x_0[10] = rtb_TX_DATA_Low_cy;
  x_0[11] = rtb_TX_DATA_Hig_dt;
  x_0[12] = rtb_TX_DATA_Low_my;
  x_0[13] = rtb_TX_DATA_Hig_k;
  x_0[14] = rtb_TX_DATA_Low_e;
  x_0[15] = rtb_TX_DATA_Hig_ik;
  x_0[16] = rtb_TX_DATA_Low_am;
  x_0[17] = rtb_TX_DATA_Hig_ex;
  x_0[18] = rtb_TX_DATA_Low_p;
  x_0[19] = rtb_TX_DATA_Hig_b;
  x_0[20] = rtb_TX_DATA_Low_ja;
  x_0[21] = rtb_TX_DATA_Hig_o;
  x_0[22] = rtb_TX_DATA_Low_jy;
  x_0[23] = rtb_TX_DATA_Hig_be;
  x_0[24] = rtb_TX_DATA_Low_pk;
  x_0[25] = rtb_TX_DATA_Hig_p;
  x_0[26] = rtb_TX_DATA_Low_c;
  x_0[27] = rtb_TX_DATA_Hig_fc;
  x_0[28] = rtb_TX_DATA_Low_ke;
  x_0[29] = rtb_TX_DATA_Hig_f5;
  x_0[30] = rtb_TX_DATA_Low_k;
  x_0[31] = rtb_TX_DATA_Hig_f;
  x_0[32] = rtb_TX_DATA_Low_m;
  x_0[33] = rtb_TX_DATA_Hig_ba;
  x_0[34] = rtb_TX_DATA_Low_k5;
  x_0[35] = rtb_TX_DATA_Hig_d;
  x_0[36] = rtb_TX_DATA_Low_by;
  x_0[37] = rtb_TX_DATA_Hig_nh;
  x_0[38] = rtb_TX_DATA_Low_nq;
  x_0[39] = rtb_TX_DATA_g;
  x_0[40] = rtb_TX_DATA_Low_cp;
  x_0[41] = rtb_TX_DATA_Hig_ge;
  x_0[42] = rtb_TX_DATA_Low_d;
  x_0[43] = rtb_TX_DATA_Hig;
  x_0[44] = rtb_TX_DATA_Low;
  x_0[45] = rtb_TX_DATA_Hig_oh;
  x_0[46] = rtb_TX_DATA_Low_eg;
  y = 18.0;
  for (k = 0; k < 46; k++) {
    y += (real_T)x_0[k + 1];
  }

  /* SignalConversion generated from: '<S8>/SCI Transmit' incorporates:
   *  Constant: '<S91>/Constant1'
   *  Constant: '<S91>/Constant2'
   *  Constant: '<S91>/Constant3'
   *  Constant: '<S91>/Constant4'
   */
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[0] = 126U;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[1] = 126U;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[2] = 47U;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[3] = 18U;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[4] =
    rtb_TX_DATA_Hig_gp;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[5] =
    rtb_TX_DATA_Low_mw;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[6] =
    SCI_Buffer;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[7] =
    rtb_TX_DATA_Low_pr;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[8] =
    rtb_TX_DATA_Hig_dl;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[9] =
    rtb_TX_DATA_Low_h;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[10] =
    rtb_TX_DATA_Hig_i3;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[11] =
    rtb_TX_DATA_Low_bd;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[12] =
    rtb_TX_DATA_Hig_nj;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[13] =
    rtb_TX_DATA_Low_cy;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[14] =
    rtb_TX_DATA_Hig_dt;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[15] =
    rtb_TX_DATA_Low_my;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[16] =
    rtb_TX_DATA_Hig_k;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[17] =
    rtb_TX_DATA_Low_e;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[18] =
    rtb_TX_DATA_Hig_ik;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[19] =
    rtb_TX_DATA_Low_am;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[20] =
    rtb_TX_DATA_Hig_ex;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[21] =
    rtb_TX_DATA_Low_p;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[22] =
    rtb_TX_DATA_Hig_b;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[23] =
    rtb_TX_DATA_Low_ja;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[24] =
    rtb_TX_DATA_Hig_o;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[25] =
    rtb_TX_DATA_Low_jy;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[26] =
    rtb_TX_DATA_Hig_be;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[27] =
    rtb_TX_DATA_Low_pk;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[28] =
    rtb_TX_DATA_Hig_p;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[29] =
    rtb_TX_DATA_Low_c;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[30] =
    rtb_TX_DATA_Hig_fc;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[31] =
    rtb_TX_DATA_Low_ke;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[32] =
    rtb_TX_DATA_Hig_f5;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[33] =
    rtb_TX_DATA_Low_k;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[34] =
    rtb_TX_DATA_Hig_f;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[35] =
    rtb_TX_DATA_Low_m;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[36] =
    rtb_TX_DATA_Hig_ba;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[37] =
    rtb_TX_DATA_Low_k5;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[38] =
    rtb_TX_DATA_Hig_d;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[39] =
    rtb_TX_DATA_Low_by;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[40] =
    rtb_TX_DATA_Hig_nh;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[41] =
    rtb_TX_DATA_Low_nq;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[42] =
    rtb_TX_DATA_g;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[43] =
    rtb_TX_DATA_Low_cp;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[44] =
    rtb_TX_DATA_Hig_ge;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[45] =
    rtb_TX_DATA_Low_d;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[46] =
    rtb_TX_DATA_Hig;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[47] =
    rtb_TX_DATA_Low;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[48] =
    rtb_TX_DATA_Hig_oh;
  Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[49] =
    rtb_TX_DATA_Low_eg;

  /* MATLAB Function: '<S91>/MATLAB Function22' */
  if (y < 1.8446744073709552E+19) {
    y_0 = (uint64_T)y;
  } else {
    y_0 = MAX_uint64_T;
  }

  y = rt_roundd((real_T)(y_0 & 255ULL));
  if (y < 256.0) {
    if (y >= 0.0) {
      /* SignalConversion generated from: '<S8>/SCI Transmit' */
      Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[50] =
        (uint16_T)y;
    } else {
      /* SignalConversion generated from: '<S8>/SCI Transmit' */
      Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[50] = 0U;
    }
  } else {
    /* SignalConversion generated from: '<S8>/SCI Transmit' */
    Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[50] = 255U;
  }

  /* S-Function (c28xsci_tx): '<S8>/SCI Transmit' */
  {
    scia_xmit((char*)
              &Fuel_Pump_V3_1_20201113_20KHz_B.TmpSignalConversionAtSCITransmi[0],
              51, 1);
  }
}

/* Model initialize function */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_initialize, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Fuel_Pump_V3_1_20201113_20KH_M, 0,
                sizeof(RT_MODEL_Fuel_Pump_V3_1_20201_T));

  /* block I/O */
  (void) memset(((void *) &Fuel_Pump_V3_1_20201113_20KHz_B), 0,
                sizeof(B_Fuel_Pump_V3_1_20201113_20K_T));

  /* states (dwork) */
  (void) memset((void *)&Fuel_Pump_V3_1_20201113_20KH_DW, 0,
                sizeof(DW_Fuel_Pump_V3_1_20201113_20_T));

  /* exported global states */
  (void) memset(&Data_Buffer_ThetaSMO, 0,
                1000U*sizeof(real32_T));
  Buffer_Out_ThetaSMO = 0.0F;
  Buffer_Out_ThetaAD2S = 0.0F;
  Buffer_Out_IqDem = 0.0F;
  Buffer_Out_Iq = 0.0F;
  Buffer_Out_Id = 0.0F;
  Buffer_Out_IdDem = 0.0F;
  n_R = 0.0F;
  n_Q = 0.0F;
  Ready_1210 = 0.0F;
  Current_W1 = 0.0F;
  Current_U1 = 0.0F;
  Current_V1 = 0.0F;
  Angle = 0.0F;
  SMO_A = 0.0F;
  SMO_K1 = 0.0F;
  SMO_K2 = 0.0F;
  nDem_PI = 0.0F;
  Uq_VVF = 0.0F;
  Ud_VVF = 0.0F;
  LWFf = 0.0F;
  n_Filter = 0.0F;
  Us = 0.0F;
  I_alpha = 0.0F;
  I_beta = 0.0F;
  SMO_KP = 0.0F;
  SMO_KI = 0.0F;
  V_alpha = 0.0F;
  V_beta = 0.0F;
  EMF_alpha = 0.0F;
  EMF_beta = 0.0F;
  SMO_uin = 0.0F;
  LWFDem = 0.0F;
  nDem = 0.0F;
  n = 0.0F;
  SetpointValue = 0.0F;
  AD_Angle = 0.0F;
  n_Step = 0.0F;
  SMO_Theta = 0.0F;
  SMO_n = 0.0F;
  TEMP_MCU1 = 0.0F;
  TEMP_MCU2 = 0.0F;
  TEMP_MOT1 = 0.0F;
  TEMP_MOT2 = 0.0F;
  en = 0.0F;
  uin = 0.0F;
  un_PI = 0.0F;
  PK_Kpn = 0.0F;
  SMO_Slide = 0.0F;
  SMO_ComP = 0.0F;
  n_1210 = 0.0F;
  PK_Kin = 0.0F;
  SMO_Filter = 0.0F;
  (void) memset(&Data_Buffer_ThetaAD2S, 0,
                1000U*sizeof(real32_T));
  IqDem = 0.0F;
  Id = 0.0F;
  (void) memset(&Data_Buffer_IqDem, 0,
                1000U*sizeof(real32_T));
  uiId = 0.0F;
  Ud = 0.0F;
  IdDem = 0.0F;
  Iq = 0.0F;
  (void) memset(&Data_Buffer_Iq, 0,
                1000U*sizeof(real32_T));
  uiIq = 0.0F;
  PK_KpIq = 0.0F;
  Uq = 0.0F;
  PK_KpId = 0.0F;
  PK_KiId = 0.0F;
  PK_KiIq = 0.0F;
  (void) memset(&Data_Buffer_IdDem, 0,
                1000U*sizeof(real32_T));
  (void) memset(&Data_Buffer_Id, 0,
                1000U*sizeof(real32_T));
  eLWFf = 0.0F;
  CPU_Timer = 0U;
  VVF_Frq = 0;
  IF_Frq = 0;
  ControlMode = 0U;
  Mid_Position = 0U;
  Buffer_Flag = 0U;
  bFlt_OvrLim_WindingUT = 0U;
  bFlt_OvrLim_WindingVT = 0U;
  bFlt_OvrLim_TempMOST = 0U;
  bFlt_OvrLim_TempUT = 0U;
  bFlt_OvrLim_TempVT = 0U;
  MCU_ST = 0U;
  MCU_IN = 0U;
  Offset_W1 = 0U;
  GF_MODE = 0U;
  SCI_Break_Error = 0U;
  TX_DATA = 0U;
  Stop = 0U;
  U1 = 0U;
  RMS_Count = 0U;
  RX_DATA = 0U;
  V1 = 0U;
  Protect_Flag = 0U;
  SCI_Error = 0U;
  NC1 = 0U;
  NC2 = 0U;
  bFlt_LWFf = 0U;
  bFlt_Resolver = 0U;
  bFlt_MOST_T = 0U;
  bFlt_WindingUT_Sensor = 0U;
  bFlt_WindingVT_Sensor = 0U;
  bFlt_Winding_I = 0U;
  bFlt_OvrLim_I = 0U;
  bFlt_OvrLim_MOST = 0U;
  W1 = 0U;
  ATEMP_MCU1 = 0U;
  ATEMP_MCU2 = 0U;
  TEMP_Flag = 0U;
  SMO_Flag = 0U;
  Offset_U1 = 0U;
  Offset_V1 = 0U;

  /* Start for S-Function (c280xgpio_di): '<S2>/Digital Input' */
  EALLOW;
  GpioCtrlRegs.GPBMUX2.all &= 0xFFFF03FF;
  GpioCtrlRegs.GPBDIR.all &= 0xFF1FFFFF;
  EDIS;

  /* Start for S-Function (c280xgpio_di): '<S2>/Digital Input1' */
  EALLOW;
  GpioCtrlRegs.GPBMUX2.all &= 0xFFFCFFFF;
  GpioCtrlRegs.GPBDIR.all &= 0xFEFFFFFF;
  EDIS;

  /* Start for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' incorporates:
   *  SubSystem: '<Root>/Motor_Control'
   */
  Fuel_Pu_Motor_Control_Start();

  /* Start for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' incorporates:
   *  SubSystem: '<Root>/IIC_Interrupt '
   */

  /* Start for function-call system: '<Root>/IIC_Interrupt ' */

  /* Start for MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  SubSystem: '<S4>/Subsystem'
   */
  /* Start for S-Function (c280xi2c_rx): '<S25>/I2C Receive' */

  /* Initialize Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[0] */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[0] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive_m[1] = (uint8_T)0.0;
  }

  /* Start for MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  SubSystem: '<S4>/Subsystem1'
   */
  /* Start for S-Function (c280xi2c_rx): '<S26>/I2C Receive' */

  /* Initialize Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[0] */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[0] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.I2CReceive[1] = (uint8_T)0.0;
  }

  /* End of Start for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */

  /* Start for S-Function (idletask): '<Root>/Idle Task' incorporates:
   *  SubSystem: '<Root>/Function-Call Subsystem1'
   */

  /* Start for function-call system: '<Root>/Function-Call Subsystem1' */
  Fuel_P_MovingAverage1_Start(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage);
  Fuel_P_MovingAverage1_Start(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage1);

  /* Start for S-Function (c280xcanrcv): '<S11>/eCAN Receive' incorporates:
   *  SubSystem: '<S11>/DTO Processing'
   */

  /* Start for function-call system: '<S11>/DTO Processing' */

  /* Start for Chart: '<S15>/Chart' incorporates:
   *  SubSystem: '<S15>/CCP(Termination)'
   */
  /* Start for function-call system: '<S15>/CCP(Termination)' */

  /* Start for S-Function (sfun_ccp_termination): '<S18>/CAN Calibration Protocol (Termination)' incorporates:
   *  SubSystem: '<S18>/CAN Transmit'
   */

  /* Start for function-call system: '<S18>/CAN Transmit' */

  /* Start for S-Function (c280xcanxmt): '<S23>/eCAN Transmit' */
  {
    /* Configure mailbox 2 to transmit messages with the ID: 1787 */
    config_eCAN_A_mbx (1U, 2, 1787, 0);
  }

  /* End of Start for S-Function (sfun_ccp_termination): '<S18>/CAN Calibration Protocol (Termination)' */

  /* Configure mailbox 0 to receive messages with the ID: 1786 */
  config_eCAN_A_mbx (0U, 0, 1786, 0);

  /* Initialize Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] */
  {
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[0] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[1] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[2] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[3] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[4] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[5] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[6] = (uint8_T)0.0;
    Fuel_Pump_V3_1_20201113_20KHz_B.eCANReceive_o2[7] = (uint8_T)0.0;
  }

  /* End of Start for S-Function (c280xcanrcv): '<S11>/eCAN Receive' */

  /* Start for S-Function (sfun_function_def): '<S14>/Function Definition' incorporates:
   *  SubSystem: '<S14>/Send DAQ Message'
   */

  /* Start for function-call system: '<S14>/Send DAQ Message' */

  /* Start for S-Function (c280xcanxmt): '<S16>/eCAN Transmit' */
  {
    /* Configure mailbox 1 to transmit messages with the ID: 1787 */
    config_eCAN_A_mbx (1U, 1, 1787, 0);
  }

  /* End of Start for S-Function (sfun_function_def): '<S14>/Function Definition' */

  /* End of Start for S-Function (idletask): '<Root>/Idle Task' */

  /* Start for DataStoreMemory: '<Root>/Data Store Memory' */
  ControlMode = 4U;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory35' */
  n_Filter = 2000.0F;

  /* Start for DataStoreMemory: '<Root>/Data Store Memory60' */
  n_Step = 0.00033333333F;

  /* Start for Atomic SubSystem: '<Root>/Initialize Function' */

  /* Start for S-Function (c280xgpio_do): '<S6>/Digital Output' incorporates:
   *  Constant: '<S6>/Constant10'
   */
  EALLOW;
  GpioCtrlRegs.GPBMUX2.all &= 0xFFFFFCFF;
  GpioCtrlRegs.GPBDIR.all |= 0x100000;
  EDIS;

  /* End of Start for SubSystem: '<Root>/Initialize Function' */

  /* user code (Initialize function Body) */

  /* System '<Root>' */

  /* Enable internal pull-up for the selected pins */
  // Pull-ups can be enabled or disabled by the user.
  // This will enable the pullups for the specified pins.
  // Comment out other unwanted lines.
  EALLOW;

  /* Enable internal pull-up for the selected pins */
  // Pull-ups can be enabled or disabled by the user.
  // This will enable the pullups for the specified pins.
  // Comment out other unwanted lines.
  GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up on GPIO16 (SPISIMOA)
  GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up on GPIO17 (SPISOMIA)
  GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up on GPIO18 (SPICLKA)

  //    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)
  /* Set qualification for selected pins to asynch only */
  // This will select asynch (no qualification) for the selected pins.
  // Comment out other unwanted lines.
  GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;// Asynch input GPIO16 (SPISIMOA)
  GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;// Asynch input GPIO17 (SPISOMIA)
  GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;// Asynch input GPIO18 (SPICLKA)

  //    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)
  /* Configure SPI-A pins using GPIO regs*/
  // This specifies which of the possible GPIO pins will be SPI functional pins.
  // Comment out other unwanted lines.
  GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
  GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
  GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA

  //    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA
  //    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; // Configure GPIO57 as SPISTEA
  //    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 0 ;// Configure GPIO57 as SPISTEA
  EDIS;
  SpiaRegs.SPICCR.all = 0x000F;       // Reset on, rising edge, 16-bit char bits
  SpiaRegs.SPICTL.all = 0x0006;        // Enable master mode, normal phase,

  // enable talk, and SPI int disabled.
  SpiaRegs.SPIBRR = 0x11;

  //    SpiaRegs.SPIBRR = 0x01;
  SpiaRegs.SPICCR.all = 0x008F;        // Relinquish SPI from Reset
  SpiaRegs.SPIPRI.bit.FREE = 1;     // Set so breakpoints don't disturb xmission
  SpiaRegs.SPIFFTX.all= 0xE060;
  SpiaRegs.SPIFFRX.all= 0x2040;
  SpiaRegs.SPIFFCT.all= 0x0;
  EALLOW;
  GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0; //RESET
  GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;
  GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0;
  GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;
  DELAY_US(2);
  GpioDataRegs.GPASET.bit.GPIO21 = 1;

  //  enter configure mode
  //  enter configure mode
  GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0; //A0=1
  GpioCtrlRegs.GPADIR.bit.GPIO10= 1;
  GpioCtrlRegs.GPAQSEL1.bit.GPIO10 = 0;
  GpioDataRegs.GPASET.bit.GPIO10 = 1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0; //A1=1
  GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
  GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 0;
  GpioDataRegs.GPASET.bit.GPIO11 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;  //wr
  GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;  //sample
  GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
  EDIS;

  //  set resolution of the normal mode
  int i;
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  for (i=0;i<100;i++) {
  }

  ;
  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;//wr=0
  SpiaRegs.SPITXBUF= 0x92;
  while (SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
  }

  RX_DATA = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  for (i=0;i<100;i++) {
  }

  ;
  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;//wr=0
  SpiaRegs.SPITXBUF= 0x7D;             //配置分辨率为14位
  while (SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
  }

  RX_DATA = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  for (i=0;i<100;i++) {
  }

  ;

  //  设置激励频率
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  for (i=0;i<100;i++) {
  }

  ;
  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;//wr=0
  SpiaRegs.SPITXBUF= 0x91;
  while (SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
  }

  RX_DATA = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  for (i=0;i<100;i++) {
  }

  ;
  GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;//wr=0
  SpiaRegs.SPITXBUF= 0x28;             //激励频率10kHz
  while (SpiaRegs.SPIFFRX.bit.RXFFST !=1) {
  }

  RX_DATA = SpiaRegs.SPIRXBUF;
  GpioDataRegs.GPASET.bit.GPIO19 = 1;  //wr=1
  GpioDataRegs.GPASET.bit.GPIO20 = 1;  //SAMPLE=1
  for (i=0;i<100;i++) {
  }

  ;
  GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;//A0=0
  GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;//A1=0
  Ready_1210 = 1;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_c = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function1' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_k = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function2' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_mo = 0.0;
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt2_m = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function3' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_h = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function4' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function5' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_UI = 0.0;
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_VI = 0.0;
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt_WI = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function7' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_m = 0.0;
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt2 = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function8' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt1_k = 0.0;

  /* SystemInitialize for MATLAB Function: '<S90>/MATLAB Function9' */
  Fuel_Pump_V3_1_20201113_20KH_DW.cnt1 = 0.0;

  /* SystemInitialize for Atomic SubSystem: '<Root>/CCP_Buffer' */
  /* SystemInitialize for MATLAB Function: '<S1>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count_out = 1.0F;
  Fuel_Pump_V3_1_20201113_20KH_DW.switch_count = 1.0F;

  /* End of SystemInitialize for SubSystem: '<Root>/CCP_Buffer' */

  /* SystemInitialize for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' incorporates:
   *  SubSystem: '<Root>/Motor_Control'
   */
  Fuel_Pum_Motor_Control_Init();

  /* End of SystemInitialize for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */

  /* SystemInitialize for S-Function (idletask): '<Root>/Idle Task' incorporates:
   *  SubSystem: '<Root>/Function-Call Subsystem1'
   */

  /* System initialize for function-call system: '<Root>/Function-Call Subsystem1' */
  Fuel_Pu_MovingAverage1_Init(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage);
  Fuel_Pu_MovingAverage1_Init(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage1);

  /* SystemInitialize for S-Function (c280xcanrcv): '<S11>/eCAN Receive' incorporates:
   *  SubSystem: '<S11>/DTO Processing'
   */

  /* System initialize for function-call system: '<S11>/DTO Processing' */

  /* SystemInitialize for Chart: '<S15>/Chart' incorporates:
   *  SubSystem: '<S15>/CCP'
   */
  /* SystemInitialize for Chart: '<S20>/CCP Stateflow ' */
  Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
    Fuel_Pump_V3_IN_NO_ACTIVE_CHILD;
  Fuel_Pump_V3_1_20201113_20KH_DW.command_counter = 0U;

  /* Chart: '<S20>/CCP Stateflow ' */
  Fuel_Pump_V3_1_20201113_20_init();
  Fuel_Pump_V3_1_20201113_20KH_DW.is_c28_canblocks_extras =
    Fuel_Pump_V3_1__IN_Disconnected;
  setCurrent_State(CCP_DISCONNECTED_STATE);
  c_reset_all_DAQ_lists();

  /* End of SystemInitialize for S-Function (c280xcanrcv): '<S11>/eCAN Receive' */

  /* End of SystemInitialize for S-Function (idletask): '<Root>/Idle Task' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/IIC_PT1000' */
  /* SystemInitialize for MATLAB Function: '<S5>/MATLAB Function' */
  Fuel_Pump_V3_1_20201113_20KH_DW.count = 1.0;

  /* End of SystemInitialize for SubSystem: '<Root>/IIC_PT1000' */

  /* SystemInitialize for Atomic SubSystem: '<Root>/Initialize Function' */
  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit' incorporates:
   *  Constant: '<S6>/0x4034'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 64;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(12352U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((12352U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit1' incorporates:
   *  Constant: '<S6>/a0x4034'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 65;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(12352U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((12352U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function1' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit3' incorporates:
   *  Constant: '<S6>/0x440A'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 64;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(2628U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((2628U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function2' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit4' incorporates:
   *  Constant: '<S6>/a0x440A'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 65;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(2628U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((2628U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function3' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit6' incorporates:
   *  Constant: '<S6>/0x4806'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 64;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(1608U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((1608U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function4' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit7' incorporates:
   *  Constant: '<S6>/a0x4806'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 65;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(1608U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((1608U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function5' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit9' incorporates:
   *  Constant: '<S6>/0x4C80'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 64;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(32844U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((32844U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function6' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit10' incorporates:
   *  Constant: '<S6>/a0x4C80'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 65;            /* Set slave address*/
      I2caRegs.I2CCNT= 2;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST>14 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)(32844U&0xFF);
        I2caRegs.I2CDXR = (uint8_T)((32844U>>8&0xFF));
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function7' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit14' incorporates:
   *  Constant: '<S6>/0x08'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 64;            /* Set slave address*/
      I2caRegs.I2CCNT= 1;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST==16 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)8U;
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function8' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (c280xi2c_tx): '<S6>/I2C Transmit12' incorporates:
   *  Constant: '<S6>/a0x08'
   */
  {
    int unsigned tx_loop= 0;
    while (I2caRegs.I2CFFTX.bit.TXFFST!=0 && tx_loop<10000 )
      tx_loop++;
    if (tx_loop!=10000) {
      I2caRegs.I2CSAR = 65;            /* Set slave address*/
      I2caRegs.I2CCNT= 1;              /* Set data length */

      /* mode:1 (1:master 0:slave)  Addressing mode:0 (1:10-bit 0:7-bit)
         free data mode:0 (1:enbaled 0:disabled) digital loopback mode:0 (1:enabled 0:disabled)
         bit count:0 (0:8bit) stop condition:1 (1:enabled 0: disabled)*/
      I2caRegs.I2CMDR.all = 28192;
      tx_loop= 0;
      while (I2caRegs.I2CFFTX.bit.TXFFST==16 && tx_loop<10000)
        tx_loop++;
      if (tx_loop!=10000) {
        I2caRegs.I2CDXR = (uint8_T)8U;
      }
    }
  }

  /* S-Function (User_Defined_Code): '<S6>/S-Function9' */
  /*CCCCC */
  /*  */
  DELAY_US(800L);

  /* S-Function (User_Defined_Code): '<S6>/S-Function10' */
  /*CCCCC */
  /*  */
  DELAY_US(8000L);

  /* S-Function (c280xgpio_do): '<S6>/Digital Output' incorporates:
   *  Constant: '<S6>/Constant10'
   */
  {
    if (0U)
      GpioDataRegs.GPBSET.bit.GPIO52 = 1;
    else
      GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
  }

  /* DataStoreWrite: '<S6>/Data Store Write' incorporates:
   *  Constant: '<S6>/Constant1'
   */
  ControlMode = 0U;

  /* DataStoreWrite: '<S6>/Data Store Write14' incorporates:
   *  Constant: '<S6>/Constant11'
   */
  SMO_KI = 253.302963F;

  /* DataStoreWrite: '<S6>/Data Store Write16' incorporates:
   *  Constant: '<S6>/Constant12'
   */
  SMO_Slide = 10.0F;

  /* DataStoreWrite: '<S6>/Data Store Write11' incorporates:
   *  Constant: '<S6>/Constant13'
   */
  PK_KpIq = 0.0341658F;

  /* DataStoreWrite: '<S6>/Data Store Write13' incorporates:
   *  Constant: '<S6>/Constant14'
   */
  PK_KiIq = 27.2727F;

  /* DataStoreWrite: '<S6>/Data Store Write3' incorporates:
   *  Constant: '<S6>/Constant15'
   */
  PK_Kpn = 0.025F;

  /* DataStoreWrite: '<S6>/Data Store Write15' incorporates:
   *  Constant: '<S6>/Constant16'
   */
  PK_Kin = 0.3F;

  /* DataStoreWrite: '<S6>/Data Store Write6' incorporates:
   *  Constant: '<S6>/Constant17'
   */
  n_Filter = 100.0F;

  /* DataStoreWrite: '<S6>/Data Store Write7' incorporates:
   *  Constant: '<S6>/Constant18'
   */
  Stop = 1U;

  /* DataStoreWrite: '<S6>/Data Store Write17' incorporates:
   *  Constant: '<S6>/Constant19'
   */
  SMO_ComP = -0.5F;

  /* DataStoreWrite: '<S6>/Data Store Write8' incorporates:
   *  Constant: '<S6>/Constant2'
   */
  n_Step = 0.002F;

  /* DataStoreWrite: '<S6>/Data Store Write19' incorporates:
   *  Constant: '<S6>/Constant20'
   */
  n_Q = 1.0F;

  /* DataStoreWrite: '<S6>/Data Store Write18' incorporates:
   *  Constant: '<S6>/Constant21'
   */
  n_R = 10.0F;

  /* DataStoreWrite: '<S6>/Data Store Write4' incorporates:
   *  Constant: '<S6>/Constant3'
   */
  SMO_A = 0.1F;

  /* DataStoreWrite: '<S6>/Data Store Write9' incorporates:
   *  Constant: '<S6>/Constant4'
   */
  SMO_K1 = 2.0F;

  /* DataStoreWrite: '<S6>/Data Store Write1' incorporates:
   *  Constant: '<S6>/Constant5'
   */
  PK_KpId = 0.0214285497F;

  /* DataStoreWrite: '<S6>/Data Store Write2' incorporates:
   *  Constant: '<S6>/Constant6'
   */
  PK_KiId = 27.2727F;

  /* DataStoreWrite: '<S6>/Data Store Write10' incorporates:
   *  Constant: '<S6>/Constant7'
   */
  SMO_K2 = 300.0F;

  /* DataStoreWrite: '<S6>/Data Store Write12' incorporates:
   *  Constant: '<S6>/Constant8'
   */
  SMO_KP = 15.915494F;

  /* DataStoreWrite: '<S6>/Data Store Write5' incorporates:
   *  Constant: '<S6>/Constant9'
   */
  Mid_Position = 193U;

  /* End of SystemInitialize for SubSystem: '<Root>/Initialize Function' */
}

/* Model terminate function */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_terminate, "ramfuncs")

void Fuel_Pump_V3_1_20201113_20KHz_terminate(void)
{
  /* Terminate for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' incorporates:
   *  SubSystem: '<Root>/Motor_Control'
   */
  Fuel_Pum_Motor_Control_Term();

  /* End of Terminate for S-Function (c28xisr_c2000): '<Root>/C28x Hardware Interrupt' */

  /* Terminate for S-Function (idletask): '<Root>/Idle Task' incorporates:
   *  SubSystem: '<Root>/Function-Call Subsystem1'
   */

  /* Termination for function-call system: '<Root>/Function-Call Subsystem1' */
  Fuel_Pu_MovingAverage1_Term(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage);
  Fuel_Pu_MovingAverage1_Term(&Fuel_Pump_V3_1_20201113_20KH_DW.MovingAverage1);

  /* End of Terminate for S-Function (idletask): '<Root>/Idle Task' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
