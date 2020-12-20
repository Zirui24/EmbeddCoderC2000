#include "Fuel_Pump_V3_1_20201113_20KHz_sfun.h"
#define PROCESS_MEX_SFUNCTION_CMD_LINE_CALL

unsigned int sf_process_check_sum_call( int nlhs, mxArray * plhs[], int nrhs,
  const mxArray * prhs[] )
{
  extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_process_check_sum_call
    ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_process_check_sum_call( int nlhs,
    mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  if (sf_Fuel_Pump_V3_1_20201113_20KHz_process_check_sum_call(nlhs,plhs,nrhs,
       prhs))
    return 1;
  if (sf_canblocks_extras_process_check_sum_call(nlhs,plhs,nrhs,prhs))
    return 1;
  return 0;
}

unsigned int sf_process_get_third_party_uses_info_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_third_party_uses_info
    ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_third_party_uses_info( int nlhs,
    mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  char commandName[64];
  char machineName[128];
  if (nrhs < 4) {
    return 0;
  }

  if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;
  mxGetString(prhs[1], machineName,sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (strcmp(machineName, "Fuel_Pump_V3_1_20201113_20KHz") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_Fuel_Pump_V3_1_20201113_20KHz_third_party_uses_info(nlhs,plhs,3,
      newRhs);
  }

  if (strcmp(machineName, "canblocks_extras") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_canblocks_extras_third_party_uses_info(nlhs,plhs,3,newRhs);
  }

  return 0;
}

unsigned int sf_process_get_jit_fallback_info_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_jit_fallback_info( int
    nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_jit_fallback_info( int nlhs, mxArray *
    plhs[], int nrhs, const mxArray * prhs[] );
  char commandName[64];
  char machineName[128];
  if (nrhs < 4) {
    return 0;
  }

  if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;
  mxGetString(prhs[1], machineName,sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (strcmp(machineName, "Fuel_Pump_V3_1_20201113_20KHz") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_Fuel_Pump_V3_1_20201113_20KHz_jit_fallback_info(nlhs,plhs,3,newRhs);
  }

  if (strcmp(machineName, "canblocks_extras") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_canblocks_extras_jit_fallback_info(nlhs,plhs,3,newRhs);
  }

  return 0;
}

unsigned int sf_process_get_post_codegen_info_call( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_get_post_codegen_info
    ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_get_post_codegen_info( int nlhs,
    mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  char commandName[64];
  char machineName[128];
  if (nrhs < 4) {
    return 0;
  }

  if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_post_codegen_info"))
    return 0;
  mxGetString(prhs[1], machineName,sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (strcmp(machineName, "Fuel_Pump_V3_1_20201113_20KHz") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_Fuel_Pump_V3_1_20201113_20KHz_get_post_codegen_info(nlhs,plhs,3,
      newRhs);
  }

  if (strcmp(machineName, "canblocks_extras") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_canblocks_extras_get_post_codegen_info(nlhs,plhs,3,newRhs);
  }

  return 0;
}

unsigned int sf_process_get_updateBuildInfo_args_info_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_updateBuildInfo_args_info
    ( int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_updateBuildInfo_args_info( int nlhs,
    mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  char commandName[64];
  char machineName[128];
  if (nrhs < 4) {
    return 0;
  }

  if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;
  mxGetString(prhs[1], machineName,sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (strcmp(machineName, "Fuel_Pump_V3_1_20201113_20KHz") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_Fuel_Pump_V3_1_20201113_20KHz_updateBuildInfo_args_info(nlhs,plhs,
      3,newRhs);
  }

  if (strcmp(machineName, "canblocks_extras") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_canblocks_extras_updateBuildInfo_args_info(nlhs,plhs,3,newRhs);
  }

  return 0;
}

unsigned int sf_process_get_eml_resolved_functions_info_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  extern unsigned int
    sf_Fuel_Pump_V3_1_20201113_20KHz_get_eml_resolved_functions_info( int nlhs,
    mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  extern unsigned int sf_canblocks_extras_get_eml_resolved_functions_info( int
    nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[] );
  char commandName[64];
  char machineName[128];
  if (nrhs < 3) {
    return 0;
  }

  if (!mxIsChar(prhs[0]) || !mxIsChar(prhs[1]))
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;
  mxGetString(prhs[1], machineName,sizeof(machineName)/sizeof(char));
  machineName[(sizeof(machineName)/sizeof(char)-1)] = '\0';
  if (strcmp(machineName, "Fuel_Pump_V3_1_20201113_20KHz") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_Fuel_Pump_V3_1_20201113_20KHz_get_eml_resolved_functions_info(nlhs,
      plhs,3,newRhs);
  }

  if (strcmp(machineName, "canblocks_extras") == 0) {
    const mxArray *newRhs[3] = { NULL, NULL, NULL };

    newRhs[0] = prhs[0];
    newRhs[1] = prhs[2];
    newRhs[2] = prhs[3];
    return sf_canblocks_extras_get_eml_resolved_functions_info(nlhs,plhs,3,
      newRhs);
  }

  return 0;
}

unsigned int sf_mex_unlock_call( int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[] )
{
  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_mex_unlock"))
    return 0;
  while (mexIsLocked()) {
    mexUnlock();
  }

  return(1);
}

static unsigned int ProcessMexSfunctionCmdLineCall(int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[])
{
  if (sf_process_check_sum_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_mex_unlock_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_process_get_third_party_uses_info_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_process_get_jit_fallback_info_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_process_get_post_codegen_info_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_process_get_updateBuildInfo_args_info_call(nlhs,plhs,nrhs,prhs))
    return 1;
  if (sf_process_get_eml_resolved_functions_info_call(nlhs,plhs,nrhs,prhs))
    return 1;
  mexErrMsgTxt("Unsuccessful command.");
  return 0;
}

static unsigned int sfGlobalMdlStartCallCounts = 0;
unsigned int sf_machine_global_initializer_called(void)
{
  return(sfGlobalMdlStartCallCounts > 0);
}

extern unsigned int sf_Fuel_Pump_V3_1_20201113_20KHz_method_dispatcher(SimStruct
  *S, unsigned int chartFileNumber, const char* specsCksum, int_T method, void
  *data);
extern unsigned int sf_canblocks_extras_method_dispatcher(SimStruct *S, unsigned
  int chartFileNumber, const char* specsCksum, int_T method, void *data);
unsigned int sf_machine_global_method_dispatcher(SimStruct *simstructPtr, const
  char *machineName, unsigned int chartFileNumber, const char* specsCksum, int_T
  method, void *data)
{
  if (!strcmp(machineName,"Fuel_Pump_V3_1_20201113_20KHz")) {
    return(sf_Fuel_Pump_V3_1_20201113_20KHz_method_dispatcher(simstructPtr,
            chartFileNumber,specsCksum,method,data));
  }

  if (!strcmp(machineName,"canblocks_extras")) {
    return(sf_canblocks_extras_method_dispatcher(simstructPtr,chartFileNumber,
            specsCksum,method,data));
  }

  return 0;
}

extern void Fuel_Pump_V3_1_20201113_20KHz_terminator(void);
extern void canblocks_extras_terminator(void);
void sf_machine_global_terminator(SimStruct* S)
{
  (void)S;
  sfGlobalMdlStartCallCounts--;
  if (sfGlobalMdlStartCallCounts == 0) {
    Fuel_Pump_V3_1_20201113_20KHz_terminator();
    canblocks_extras_terminator();
  }

  return;
}

extern void Fuel_Pump_V3_1_20201113_20KHz_initializer(void);
extern void canblocks_extras_initializer(void);
extern void Fuel_Pump_V3_1_20201113_20KHz_register_exported_symbols(SimStruct* S);
extern void canblocks_extras_register_exported_symbols(SimStruct* S);
void sf_register_machine_exported_symbols(SimStruct* S)
{
  Fuel_Pump_V3_1_20201113_20KHz_register_exported_symbols(S);
  canblocks_extras_register_exported_symbols(S);
}

bool callCustomFcn(char initFlag)
{
  return false;
}

void sf_machine_global_initializer(SimStruct* S)
{
  bool simModeIsRTWGen = sim_mode_is_rtw_gen(S);
  sfGlobalMdlStartCallCounts++;
  if (sfGlobalMdlStartCallCounts == 1) {
    if (simModeIsRTWGen) {
      sf_register_machine_exported_symbols(S);
    }

    Fuel_Pump_V3_1_20201113_20KHz_initializer();
    canblocks_extras_initializer();
  }

  return;
}

#include "sf_runtime/stateflow_mdl_methods.stub"