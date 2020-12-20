/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Fuel_Pump_V3_1_20201113_20KHz_private.h
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

#ifndef RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_private_h_
#define RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_private_h_
#include "ccp_utils.h"
#include "ccp_defines.h"
#include "rtwtypes.h"
#include "can_message.h"
#include "can_message.h"
#include "Fuel_Pump_V3_1_20201113_20KHz.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFFFU) ) || ( SCHAR_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */
void config_eCAN_A_mbx (uint16_T mbxType, uint16_T mbxNo, uint32_T msgID,
  uint16_T msgType);
void config_ADC_A(uint16_T maxconvReg , uint16_T adcChselSEQ1Reg, uint16_T
                  adcChselSEQ2Reg, uint16_T adcChselSEQ3Reg, uint16_T
                  adcChselSEQ4Reg);
extern CAN_DATATYPE CAN_DATATYPE_GROUND;
extern CAN_DATATYPE CAN_DATATYPE_GROUND;
extern CAN_DATATYPE CAN_DATATYPE_GROUND;
extern CAN_DATATYPE CAN_DATATYPE_GROUND;
extern real32_T rt_roundf(real32_T u);
extern real_T rt_roundd(real_T u);

/* S-Function Block: <S14>/Function Definition */
void exported_ccp_daq_trigger(uint8_T * msg_pointer);
void isr_int1pie6_task_fcn(void);
void isr_int8pie1_task_fcn(void);
extern void configureGPIOExtInterrupt(void);
void idle_num1_task_fcn(void);
extern real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);

#pragma CODE_SECTION (Fuel_Pump_V3_1_2020111_Stop, "ramfuncs")

extern void Fuel_Pump_V3_1_2020111_Stop(void);

#pragma CODE_SECTION (Fuel_EmbeddedMATLABFunction, "ramfuncs")

extern void Fuel_EmbeddedMATLABFunction(int32_T *rty_y);

#pragma CODE_SECTION (Fuel_Pump_V3_1_20_Temp_conv, "ramfuncs")

extern void Fuel_Pump_V3_1_20_Temp_conv(uint16_T rtu_u1, uint16_T rtu_u2,
  real32_T *rty_y);

#pragma CODE_SECTION (Fuel_Pu_MovingAverage1_Init, "ramfuncs")

extern void Fuel_Pu_MovingAverage1_Init(DW_MovingAverage1_Fuel_Pump_V_T *localDW);

#pragma CODE_SECTION (Fuel_P_MovingAverage1_Reset, "ramfuncs")

extern void Fuel_P_MovingAverage1_Reset(DW_MovingAverage1_Fuel_Pump_V_T *localDW);

#pragma CODE_SECTION (Fuel_P_MovingAverage1_Start, "ramfuncs")

extern void Fuel_P_MovingAverage1_Start(DW_MovingAverage1_Fuel_Pump_V_T *localDW);

#pragma CODE_SECTION (Fuel_Pump_V3_MovingAverage1, "ramfuncs")

extern void Fuel_Pump_V3_MovingAverage1(real32_T rtu_0,
  B_MovingAverage1_Fuel_Pump_V3_T *localB, DW_MovingAverage1_Fuel_Pump_V_T
  *localDW);

#pragma CODE_SECTION (Fuel_Pump_V3_1_202_READTemp, "ramfuncs")

extern void Fuel_Pump_V3_1_202_READTemp(uint16_T rtu_AD, real32_T *rty_TEMP);

#pragma CODE_SECTION (Fuel_Pump_V_MATLABFunction1, "ramfuncs")

extern void Fuel_Pump_V_MATLABFunction1(real32_T rtu_RAW_DATA, uint16_T
  *rty_TX_DATA);

#pragma CODE_SECTION (Fuel_Pump__MATLABFunction11, "ramfuncs")

extern void Fuel_Pump__MATLABFunction11(real32_T rtu_RAW_DATA, uint16_T
  *rty_TX_DATA);

#pragma CODE_SECTION (Fuel_Pump__MATLABFunction14, "ramfuncs")

extern void Fuel_Pump__MATLABFunction14(real32_T rtu_RAW_DATA, uint16_T
  *rty_TX_DATA);

#pragma CODE_SECTION (Fuel_Pump__MATLABFunction19, "ramfuncs")

extern void Fuel_Pump__MATLABFunction19(uint16_T rtu_RAW_DATA, uint16_T
  *rty_TX_DATA);

#pragma CODE_SECTION (Fuel_Pump_V_MATLABFunction2, "ramfuncs")

extern void Fuel_Pump_V_MATLABFunction2(real32_T rtu_RAW_DATA, uint16_T
  *rty_TX_DATA);

#pragma CODE_SECTION (Fuel_Pump__MATLABFunction23, "ramfuncs")

extern void Fuel_Pump__MATLABFunction23(uint16_T rtu_TX_DATA, uint16_T
  *rty_TX_DATA_Hig, uint16_T *rty_TX_DATA_Low);

#pragma CODE_SECTION (Fuel_Pum_Motor_Control_Init, "ramfuncs")

extern void Fuel_Pum_Motor_Control_Init(void);

#pragma CODE_SECTION (Fuel_Pu_Motor_Control_Start, "ramfuncs")

extern void Fuel_Pu_Motor_Control_Start(void);

#pragma CODE_SECTION (Fuel_Pump_V3__Motor_Control, "ramfuncs")

extern void Fuel_Pump_V3__Motor_Control(void);

#pragma CODE_SECTION (Fuel_Pump_V3_1_202_Set_nDem, "ramfuncs")

extern void Fuel_Pump_V3_1_202_Set_nDem(void);

#pragma CODE_SECTION (Fuel_Pum_Motor_Control_Term, "ramfuncs")

extern void Fuel_Pum_Motor_Control_Term(void);

#pragma CODE_SECTION (Fuel_Pu_MovingAverage1_Term, "ramfuncs")

extern void Fuel_Pu_MovingAverage1_Term(DW_MovingAverage1_Fuel_Pump_V_T *localDW);
void isr_int1pie6_task_fcn(void);

#endif                 /* RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
