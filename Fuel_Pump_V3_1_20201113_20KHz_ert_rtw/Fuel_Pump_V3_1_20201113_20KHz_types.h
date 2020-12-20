/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Fuel_Pump_V3_1_20201113_20KHz_types.h
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

#ifndef RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_types_h_
#define RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_types_h_
#include "rtwtypes.h"
#ifndef typedef_e_dsp_private_SlidingWindowAv_T
#define typedef_e_dsp_private_SlidingWindowAv_T

typedef struct {
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real32_T pCumSum;
  real32_T pCumSumRev[39];
  real32_T pCumRevIndex;
} e_dsp_private_SlidingWindowAv_T;

#endif                               /*typedef_e_dsp_private_SlidingWindowAv_T*/

#ifndef typedef_c_cell_wrap_Fuel_Pump_V3_1_20_T
#define typedef_c_cell_wrap_Fuel_Pump_V3_1_20_T

typedef struct {
  uint32_T f1[8];
} c_cell_wrap_Fuel_Pump_V3_1_20_T;

#endif                               /*typedef_c_cell_wrap_Fuel_Pump_V3_1_20_T*/

#ifndef typedef_dsp_simulink_MovingAverage_Fu_T
#define typedef_dsp_simulink_MovingAverage_Fu_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  c_cell_wrap_Fuel_Pump_V3_1_20_T inputVarSize;
  e_dsp_private_SlidingWindowAv_T *pStatistic;
  int32_T NumChannels;
} dsp_simulink_MovingAverage_Fu_T;

#endif                               /*typedef_dsp_simulink_MovingAverage_Fu_T*/

/* Custom Type definition for MATLABSystem: '<S52>/SPI_Master_Transfer' */
#include "MW_SVD.h"
#ifndef typedef_codertarget_tic2000_blocks_SP_T
#define typedef_codertarget_tic2000_blocks_SP_T

typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_SPI_HANDLE;
} codertarget_tic2000_blocks_SP_T;

#endif                               /*typedef_codertarget_tic2000_blocks_SP_T*/

/* Forward declaration for rtModel */
typedef struct tag_RTM_Fuel_Pump_V3_1_202011_T RT_MODEL_Fuel_Pump_V3_1_20201_T;

#endif                   /* RTW_HEADER_Fuel_Pump_V3_1_20201113_20KHz_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
