/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Fuel_Pump_V3_1_20201113_20KHz.h
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

#ifndef RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_h_
#define RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef Fuel_Pump_V3_1_20201113_20KHz_COMMON_INCLUDES_
# define Fuel_Pump_V3_1_20201113_20KHz_COMMON_INCLUDES_
#include <IQmathLib.h>
#include <string.h>
#include "rtwtypes.h"
#include "c2000BoardSupport.h"
#include "DSP2833x_Device.h"
#include "DSP2833x_Gpio.h"
#include "DSP2833x_Examples.h"
#include "IQmathLib.h"
#include "can_message.h"
#include "DSP28xx_SciUtil.h"
#include "MW_SPI.h"
#include "MW_c2000SPI.h"
#endif                      /* Fuel_Pump_V3_1_20201113_20KHz_COMMON_INCLUDES_ */

#include "Fuel_Pump_V3_1_20201113_20KHz_types.h"
#include "MW_target_hardware_resources.h"
#include "IQmathLib.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

#define Fuel_Pump_V3_1_20201113_20KHz_M (Fuel_Pump_V3_1_20201113_20KH_M)

extern void init_eCAN_A ( uint16_T bitRatePrescaler, uint16_T timeSeg1, uint16_T
  timeSeg2, uint16_T sbg, uint16_T sjw, uint16_T sam);
extern void init_SCI(void);
extern void init_SCI_GPIO(void);
extern void config_ePWM_GPIO (void);
extern void init_I2C_GPIO(void);
extern void init_I2C_A(void);

/* user code (top of export header file) */
#include "can_message.h"
#include "can_message.h"

/* Block signals for system '<S3>/Moving Average1' */
typedef struct {
  real32_T MovingAverage1;             /* '<S3>/Moving Average1' */
} B_MovingAverage1_Fuel_Pump_V3_T;

/* Block states (default storage) for system '<S3>/Moving Average1' */
typedef struct {
  e_dsp_private_SlidingWindowAv_T gobj_0;/* '<S3>/Moving Average1' */
  e_dsp_private_SlidingWindowAv_T gobj_1;/* '<S3>/Moving Average1' */
  dsp_simulink_MovingAverage_Fu_T obj; /* '<S3>/Moving Average1' */
  boolean_T objisempty;                /* '<S3>/Moving Average1' */
} DW_MovingAverage1_Fuel_Pump_V_T;

/* Block signals (default storage) */
typedef struct {
  real32_T Add3;                       /* '<S57>/Add3' */
  int32_T RampControl_o1;              /* '<S51>/Ramp Control' */
  int32_T RampControl_o1_o;            /* '<S39>/Ramp Control' */
  int32_T RampControl_o2;              /* '<S51>/Ramp Control' */
  int32_T RampControl_o2_h;            /* '<S39>/Ramp Control' */
  uint16_T DigitalInput[3];            /* '<S2>/Digital Input' */
  uint16_T DigitalInput1;              /* '<S2>/Digital Input1' */
  uint16_T ADC[5];                     /* '<S7>/ADC' */
  uint16_T eCANReceive_o2[8];          /* '<S11>/eCAN Receive' */
  uint16_T CANCalibrationProtocolTerminati[8];
                            /* '<S18>/CAN Calibration Protocol (Termination)' */
  uint16_T I2CReceive[2];              /* '<S26>/I2C Receive' */
  uint16_T I2CReceive_m[2];            /* '<S25>/I2C Receive' */
  uint16_T TmpSignalConversionAtSCITransmi[51];/* '<S8>/SCI_TX' */
  uint16_T InputDriver[8];             /* '<S16>/Input Driver' */
  B_MovingAverage1_Fuel_Pump_V3_T MovingAverage;/* '<S3>/Moving Average1' */
  B_MovingAverage1_Fuel_Pump_V3_T MovingAverage1;/* '<S3>/Moving Average1' */
} B_Fuel_Pump_V3_1_20201113_20K_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  codertarget_tic2000_blocks_SP_T obj; /* '<S52>/SPI_Master_Transfer' */
  real_T cnt1;                         /* '<S90>/MATLAB Function9' */
  real_T cnt1_k;                       /* '<S90>/MATLAB Function8' */
  real_T cnt1_m;                       /* '<S90>/MATLAB Function7' */
  real_T cnt2;                         /* '<S90>/MATLAB Function7' */
  real_T cnt_UI;                       /* '<S90>/MATLAB Function5' */
  real_T cnt_VI;                       /* '<S90>/MATLAB Function5' */
  real_T cnt_WI;                       /* '<S90>/MATLAB Function5' */
  real_T cnt;                          /* '<S90>/MATLAB Function4' */
  real_T cnt_h;                        /* '<S90>/MATLAB Function3' */
  real_T cnt1_mo;                      /* '<S90>/MATLAB Function2' */
  real_T cnt2_m;                       /* '<S90>/MATLAB Function2' */
  real_T cnt_k;                        /* '<S90>/MATLAB Function1' */
  real_T cnt_c;                        /* '<S90>/MATLAB Function' */
  real_T count;                        /* '<S5>/MATLAB Function' */
  real_T count_d;                      /* '<S49>/MATLAB Function' */
  real_T count_d3;                     /* '<S47>/MATLAB Function' */
  real32_T Delay1_DSTATE;              /* '<S53>/Delay1' */
  real32_T UnitDelay3_DSTATE;          /* '<S57>/Unit Delay3' */
  real32_T UnitDelay1_DSTATE;          /* '<S35>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE;          /* '<S35>/Unit Delay2' */
  real32_T UnitDelay6_DSTATE;          /* '<S35>/Unit Delay6' */
  int32_T UnitDelay_DSTATE;            /* '<S84>/Unit Delay' */
  int32_T UnitDelay_DSTATE_p;          /* '<S59>/Unit Delay' */
  real32_T count_out;                  /* '<S1>/MATLAB Function' */
  real32_T switch_count;               /* '<S1>/MATLAB Function' */
  real32_T SMO_Count;                  /* '<S49>/MATLAB Function1' */
  real32_T count_a;                    /* '<S41>/MATLAB Function1' */
  int32_T RampControl_RAMP_DLY_CNTL;   /* '<S51>/Ramp Control' */
  int32_T RampControl_PREV_SETPOINT;   /* '<S51>/Ramp Control' */
  int32_T RampControl_RAMP_DLY_CNTL_p; /* '<S39>/Ramp Control' */
  int32_T RampControl_PREV_SETPOINT_n; /* '<S39>/Ramp Control' */
  uint16_T UnitDelay_DSTATE_c;         /* '<S52>/Unit Delay' */
  uint16_T UnitDelay_DSTATE_l;         /* '<S33>/Unit Delay' */
  uint16_T AD2S_n_Count;               /* '<S52>/MATLAB Function' */
  uint16_T is_c28_canblocks_extras;    /* '<S20>/CCP Stateflow ' */
  uint16_T command_counter;            /* '<S20>/CCP Stateflow ' */
  DW_MovingAverage1_Fuel_Pump_V_T MovingAverage;/* '<S3>/Moving Average1' */
  DW_MovingAverage1_Fuel_Pump_V_T MovingAverage1;/* '<S3>/Moving Average1' */
} DW_Fuel_Pump_V3_1_20201113_20_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S91>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData[11];

  /* Computed Parameter: uDLookupTable1_bp01Data
   * Referenced by: '<S91>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_bp01Data[11];
} ConstP_Fuel_Pump_V3_1_2020111_T;

/* Real-time Model Data Structure */
struct tag_RTM_Fuel_Pump_V3_1_202011_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[4];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_Fuel_Pump_V3_1_20201113_20K_T Fuel_Pump_V3_1_20201113_20KHz_B;

/* Block states (default storage) */
extern DW_Fuel_Pump_V3_1_20201113_20_T Fuel_Pump_V3_1_20201113_20KH_DW;

/* Constant parameters (default storage) */
extern const ConstP_Fuel_Pump_V3_1_2020111_T Fuel_Pump_V3_1_20201113__ConstP;

/*
 * Exported States
 *
 * Note: Exported states are block states with an exported global
 * storage class designation.  Code generation will declare the memory for these
 * states and exports their symbols.
 *
 */
extern real32_T Data_Buffer_ThetaSMO[1000];/* '<Root>/Data Store Memory100' */
extern real32_T Buffer_Out_ThetaSMO;   /* '<Root>/Data Store Memory102' */
extern real32_T Buffer_Out_ThetaAD2S;  /* '<Root>/Data Store Memory111' */
extern real32_T Buffer_Out_IqDem;      /* '<Root>/Data Store Memory114' */
extern real32_T Buffer_Out_Iq;         /* '<Root>/Data Store Memory115' */
extern real32_T Buffer_Out_Id;         /* '<Root>/Data Store Memory116' */
extern real32_T Buffer_Out_IdDem;      /* '<Root>/Data Store Memory117' */
extern real32_T n_R;                   /* '<Root>/Data Store Memory118' */
extern real32_T n_Q;                   /* '<Root>/Data Store Memory119' */
extern real32_T Ready_1210;            /* '<Root>/Data Store Memory12' */
extern real32_T Current_W1;            /* '<Root>/Data Store Memory14' */
extern real32_T Current_U1;            /* '<Root>/Data Store Memory17' */
extern real32_T Current_V1;            /* '<Root>/Data Store Memory18' */
extern real32_T Angle;                 /* '<Root>/Data Store Memory20' */
extern real32_T SMO_A;                 /* '<Root>/Data Store Memory22' */
extern real32_T SMO_K1;                /* '<Root>/Data Store Memory23' */
extern real32_T SMO_K2;                /* '<Root>/Data Store Memory24' */
extern real32_T nDem_PI;               /* '<Root>/Data Store Memory27' */
extern real32_T Uq_VVF;                /* '<Root>/Data Store Memory28' */
extern real32_T Ud_VVF;                /* '<Root>/Data Store Memory29' */
extern real32_T LWFf;                  /* '<Root>/Data Store Memory34' */
extern real32_T n_Filter;              /* '<Root>/Data Store Memory35' */
extern real32_T Us;                    /* '<Root>/Data Store Memory36' */
extern real32_T I_alpha;               /* '<Root>/Data Store Memory46' */
extern real32_T I_beta;                /* '<Root>/Data Store Memory48' */
extern real32_T SMO_KP;                /* '<Root>/Data Store Memory49' */
extern real32_T SMO_KI;                /* '<Root>/Data Store Memory50' */
extern real32_T V_alpha;               /* '<Root>/Data Store Memory51' */
extern real32_T V_beta;                /* '<Root>/Data Store Memory52' */
extern real32_T EMF_alpha;             /* '<Root>/Data Store Memory53' */
extern real32_T EMF_beta;              /* '<Root>/Data Store Memory54' */
extern real32_T SMO_uin;               /* '<Root>/Data Store Memory55' */
extern real32_T LWFDem;                /* '<Root>/Data Store Memory56' */
extern real32_T nDem;                  /* '<Root>/Data Store Memory57' */
extern real32_T n;                     /* '<Root>/Data Store Memory58' */
extern real32_T SetpointValue;         /* '<Root>/Data Store Memory59' */
extern real32_T AD_Angle;              /* '<Root>/Data Store Memory6' */
extern real32_T n_Step;                /* '<Root>/Data Store Memory60' */
extern real32_T SMO_Theta;             /* '<Root>/Data Store Memory63' */
extern real32_T SMO_n;                 /* '<Root>/Data Store Memory64' */
extern real32_T TEMP_MCU1;             /* '<Root>/Data Store Memory65' */
extern real32_T TEMP_MCU2;             /* '<Root>/Data Store Memory66' */
extern real32_T TEMP_MOT1;             /* '<Root>/Data Store Memory67' */
extern real32_T TEMP_MOT2;             /* '<Root>/Data Store Memory68' */
extern real32_T en;                    /* '<Root>/Data Store Memory71' */
extern real32_T uin;                   /* '<Root>/Data Store Memory72' */
extern real32_T un_PI;                 /* '<Root>/Data Store Memory73' */
extern real32_T PK_Kpn;                /* '<Root>/Data Store Memory74' */
extern real32_T SMO_Slide;             /* '<Root>/Data Store Memory75' */
extern real32_T SMO_ComP;              /* '<Root>/Data Store Memory76' */
extern real32_T n_1210;                /* '<Root>/Data Store Memory77' */
extern real32_T PK_Kin;                /* '<Root>/Data Store Memory78' */
extern real32_T SMO_Filter;            /* '<Root>/Data Store Memory79' */
extern real32_T Data_Buffer_ThetaAD2S[1000];/* '<Root>/Data Store Memory80' */
extern real32_T IqDem;                 /* '<Root>/Data Store Memory81' */
extern real32_T Id;                    /* '<Root>/Data Store Memory82' */
extern real32_T Data_Buffer_IqDem[1000];/* '<Root>/Data Store Memory83' */
extern real32_T uiId;                  /* '<Root>/Data Store Memory84' */
extern real32_T Ud;                    /* '<Root>/Data Store Memory86' */
extern real32_T IdDem;                 /* '<Root>/Data Store Memory87' */
extern real32_T Iq;                    /* '<Root>/Data Store Memory88' */
extern real32_T Data_Buffer_Iq[1000];  /* '<Root>/Data Store Memory89' */
extern real32_T uiIq;                  /* '<Root>/Data Store Memory90' */
extern real32_T PK_KpIq;               /* '<Root>/Data Store Memory92' */
extern real32_T Uq;                    /* '<Root>/Data Store Memory93' */
extern real32_T PK_KpId;               /* '<Root>/Data Store Memory94' */
extern real32_T PK_KiId;               /* '<Root>/Data Store Memory95' */
extern real32_T PK_KiIq;               /* '<Root>/Data Store Memory96' */
extern real32_T Data_Buffer_IdDem[1000];/* '<Root>/Data Store Memory97' */
extern real32_T Data_Buffer_Id[1000];  /* '<Root>/Data Store Memory98' */
extern real32_T eLWFf;                 /* '<Root>/Data Store Memory99' */
extern uint32_T CPU_Timer;             /* '<Root>/Data Store Memory112' */
extern int32_T VVF_Frq;                /* '<Root>/Data Store Memory4' */
extern int32_T IF_Frq;                 /* '<Root>/Data Store Memory47' */
extern uint16_T ControlMode;           /* '<Root>/Data Store Memory' */
extern uint16_T Mid_Position;          /* '<Root>/Data Store Memory1' */
extern uint16_T Buffer_Flag;           /* '<Root>/Data Store Memory101' */
extern uint16_T bFlt_OvrLim_WindingUT; /* '<Root>/Data Store Memory103' */
extern uint16_T bFlt_OvrLim_WindingVT; /* '<Root>/Data Store Memory104' */
extern uint16_T bFlt_OvrLim_TempMOST;  /* '<Root>/Data Store Memory105' */
extern uint16_T bFlt_OvrLim_TempUT;    /* '<Root>/Data Store Memory106' */
extern uint16_T bFlt_OvrLim_TempVT;    /* '<Root>/Data Store Memory107' */
extern uint16_T MCU_ST;                /* '<Root>/Data Store Memory108' */
extern uint16_T MCU_IN;                /* '<Root>/Data Store Memory109' */
extern uint16_T Offset_W1;             /* '<Root>/Data Store Memory11' */
extern uint16_T GF_MODE;               /* '<Root>/Data Store Memory110' */
extern uint16_T SCI_Break_Error;       /* '<Root>/Data Store Memory113' */
extern uint16_T TX_DATA;               /* '<Root>/Data Store Memory16' */
extern uint16_T Stop;                  /* '<Root>/Data Store Memory19' */
extern uint16_T U1;                    /* '<Root>/Data Store Memory2' */
extern uint16_T RMS_Count;             /* '<Root>/Data Store Memory21' */
extern uint16_T RX_DATA;               /* '<Root>/Data Store Memory25' */
extern uint16_T V1;                    /* '<Root>/Data Store Memory3' */
extern uint16_T Protect_Flag;          /* '<Root>/Data Store Memory30' */
extern uint16_T SCI_Error;             /* '<Root>/Data Store Memory31' */
extern uint16_T NC1;                   /* '<Root>/Data Store Memory32' */
extern uint16_T NC2;                   /* '<Root>/Data Store Memory37' */
extern uint16_T bFlt_LWFf;             /* '<Root>/Data Store Memory38' */
extern uint16_T bFlt_Resolver;         /* '<Root>/Data Store Memory39' */
extern uint16_T bFlt_MOST_T;           /* '<Root>/Data Store Memory40' */
extern uint16_T bFlt_WindingUT_Sensor; /* '<Root>/Data Store Memory41' */
extern uint16_T bFlt_WindingVT_Sensor; /* '<Root>/Data Store Memory42' */
extern uint16_T bFlt_Winding_I;        /* '<Root>/Data Store Memory43' */
extern uint16_T bFlt_OvrLim_I;         /* '<Root>/Data Store Memory44' */
extern uint16_T bFlt_OvrLim_MOST;      /* '<Root>/Data Store Memory45' */
extern uint16_T W1;                    /* '<Root>/Data Store Memory5' */
extern uint16_T ATEMP_MCU1;            /* '<Root>/Data Store Memory61' */
extern uint16_T ATEMP_MCU2;            /* '<Root>/Data Store Memory62' */
extern uint16_T TEMP_Flag;             /* '<Root>/Data Store Memory69' */
extern uint16_T SMO_Flag;              /* '<Root>/Data Store Memory70' */
extern uint16_T Offset_U1;             /* '<Root>/Data Store Memory8' */
extern uint16_T Offset_V1;             /* '<Root>/Data Store Memory9' */

/* External function called from main */
extern void Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

/* Model entry point functions */
#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_SetEventsForThisBaseStep(boolean_T
  *eventFlags);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_initialize, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_initialize(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step0, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_step0(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step1, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_step1(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step2, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_step2(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_step3, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_step3(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20201113_20KHz_terminate, "ramfuncs")

extern void Fuel_Pump_V3_1_20201113_20KHz_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Fuel_Pump_V3_1_20201_T *const Fuel_Pump_V3_1_20201113_20KH_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S64>/Data Type Duplicate' : Unused code path elimination
 * Block '<S89>/Data Type Duplicate' : Unused code path elimination
 * Block '<S64>/Conversion' : Eliminate redundant data type conversion
 * Block '<S89>/Conversion' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Fuel_Pump_V3_1_20201113_20KHz'
 * '<S1>'   : 'Fuel_Pump_V3_1_20201113_20KHz/CCP_Buffer'
 * '<S2>'   : 'Fuel_Pump_V3_1_20201113_20KHz/Extra Control '
 * '<S3>'   : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1'
 * '<S4>'   : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt '
 * '<S5>'   : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_PT1000'
 * '<S6>'   : 'Fuel_Pump_V3_1_20201113_20KHz/Initialize Function'
 * '<S7>'   : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control'
 * '<S8>'   : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX'
 * '<S9>'   : 'Fuel_Pump_V3_1_20201113_20KHz/Set_nDem'
 * '<S10>'  : 'Fuel_Pump_V3_1_20201113_20KHz/CCP_Buffer/MATLAB Function'
 * '<S11>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol'
 * '<S12>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/READ Temp'
 * '<S13>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/READ Temp1'
 * '<S14>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DAQ Processing'
 * '<S15>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing'
 * '<S16>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DAQ Processing/Send DAQ Message'
 * '<S17>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP'
 * '<S18>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP(Termination)'
 * '<S19>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/Chart'
 * '<S20>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP/CAN Calibration Protocol'
 * '<S21>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP/CAN Calibration Protocol/CCP Stateflow '
 * '<S22>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP/CAN Calibration Protocol/Interpret Data'
 * '<S23>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Function-Call Subsystem1/CAN Calibration Protocol/DTO Processing/CCP(Termination)/CAN Transmit'
 * '<S24>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt /MATLAB Function'
 * '<S25>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt /Subsystem'
 * '<S26>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt /Subsystem1'
 * '<S27>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt /Subsystem/Temp_conv'
 * '<S28>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_Interrupt /Subsystem1/Temp_conv'
 * '<S29>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_PT1000/MATLAB Function'
 * '<S30>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_PT1000/Subsystem4'
 * '<S31>'  : 'Fuel_Pump_V3_1_20201113_20KHz/IIC_PT1000/Subsystem5'
 * '<S32>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Initial Case'
 * '<S33>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Interrupt_Count'
 * '<S34>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case '
 * '<S35>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Offset Case '
 * '<S36>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210'
 * '<S37>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Adjusting Mode'
 * '<S38>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Clark Transformation'
 * '<S39>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode'
 * '<S40>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI'
 * '<S41>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Data Buffer '
 * '<S42>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Error'
 * '<S43>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /If Action Subsystem'
 * '<S44>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /MATLAB Function'
 * '<S45>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /MATLAB Function3'
 * '<S46>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Park Transformation'
 * '<S47>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode'
 * '<S48>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /SVPWM'
 * '<S49>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode'
 * '<S50>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Stop'
 * '<S51>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode'
 * '<S52>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem'
 * '<S53>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/If Action Subsystem'
 * '<S54>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/MATLAB Function'
 * '<S55>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/MATLAB Function1'
 * '<S56>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/RAW_DATA2Angle '
 * '<S57>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/If Action Subsystem/First Order Filter'
 * '<S58>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /AD2S1210/If Action Subsystem/If Action Subsystem/MATLAB Function1'
 * '<S59>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1'
 * '<S60>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1/Convert Param To fix-pt with floor  rounding mode'
 * '<S61>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1/Subsystem'
 * '<S62>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1/Subsystem1'
 * '<S63>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1/Convert Param To fix-pt with floor  rounding mode/Embedded MATLAB Function'
 * '<S64>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current Loop Mode/Ramp Generator1/Subsystem/Data Type Conversion Inherited'
 * '<S65>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI/MATLAB Function1'
 * '<S66>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI/PID_Id '
 * '<S67>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI/PID_Iq'
 * '<S68>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI/PID_Id /Integral '
 * '<S69>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Current_PI/PID_Iq/Integral '
 * '<S70>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Data Buffer /MATLAB Function1'
 * '<S71>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/If Action Subsystem'
 * '<S72>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/MATLAB Function'
 * '<S73>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/If Action Subsystem/PID_Loop'
 * '<S74>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/If Action Subsystem/PID_Speed'
 * '<S75>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/If Action Subsystem/PID_Loop/UI_REG3'
 * '<S76>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Position Loop Mode/If Action Subsystem/PID_Speed/UI_REG3'
 * '<S77>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /SVPWM/Inverse Park Transformation'
 * '<S78>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /SVPWM/MATLAB Function'
 * '<S79>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode/If Action Subsystem'
 * '<S80>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode/MATLAB Function'
 * '<S81>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode/MATLAB Function1'
 * '<S82>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode/If Action Subsystem/PID_n '
 * '<S83>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /Speed Loop Mode/If Action Subsystem/PID_n /Integral '
 * '<S84>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1'
 * '<S85>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1/Convert Param To fix-pt with floor  rounding mode'
 * '<S86>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1/Subsystem'
 * '<S87>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1/Subsystem1'
 * '<S88>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1/Convert Param To fix-pt with floor  rounding mode/Embedded MATLAB Function'
 * '<S89>'  : 'Fuel_Pump_V3_1_20201113_20KHz/Motor_Control/Main Case /VVF Mode/Ramp Generator1/Subsystem/Data Type Conversion Inherited'
 * '<S90>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM'
 * '<S91>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX'
 * '<S92>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function'
 * '<S93>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function1'
 * '<S94>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function2'
 * '<S95>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function3'
 * '<S96>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function4'
 * '<S97>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function5'
 * '<S98>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function6'
 * '<S99>'  : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function7'
 * '<S100>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function8'
 * '<S101>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/FPM/MATLAB Function9'
 * '<S102>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function'
 * '<S103>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function1'
 * '<S104>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function10'
 * '<S105>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function11'
 * '<S106>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function12'
 * '<S107>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function13'
 * '<S108>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function14'
 * '<S109>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function15'
 * '<S110>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function16'
 * '<S111>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function17'
 * '<S112>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function18'
 * '<S113>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function19'
 * '<S114>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function2'
 * '<S115>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function20'
 * '<S116>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function21'
 * '<S117>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function22'
 * '<S118>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function23'
 * '<S119>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function24'
 * '<S120>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function25'
 * '<S121>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function26'
 * '<S122>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function27'
 * '<S123>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function28'
 * '<S124>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function29'
 * '<S125>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function3'
 * '<S126>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function30'
 * '<S127>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function31'
 * '<S128>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function32'
 * '<S129>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function33'
 * '<S130>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function34'
 * '<S131>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function35'
 * '<S132>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function36'
 * '<S133>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function37'
 * '<S134>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function38'
 * '<S135>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function39'
 * '<S136>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function4'
 * '<S137>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function40'
 * '<S138>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function41'
 * '<S139>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function42'
 * '<S140>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function43'
 * '<S141>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function44'
 * '<S142>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function5'
 * '<S143>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function6'
 * '<S144>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function7'
 * '<S145>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function8'
 * '<S146>' : 'Fuel_Pump_V3_1_20201113_20KHz/SCI_TX/SCI_TX/MATLAB Function9'
 * '<S147>' : 'Fuel_Pump_V3_1_20201113_20KHz/Set_nDem/MATLAB Function1'
 */
#endif                         /* RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
