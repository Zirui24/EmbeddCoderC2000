# ------------------- Required for MSVC nmake ---------------------------------
# This file should be included at the top of a MAKEFILE as follows:


CPU = AMD64

MODEL     = Fuel_Pump_V3_1_20201113_20KHz
TARGET      = sfun
MODULE_SRCS   = 
MODEL_SRC  = Fuel_Pump_V3_1_20201113_20KHz_sfun.c
MODEL_REG = Fuel_Pump_V3_1_20201113_20KHz_sfun_registry.c
MAKEFILE    = Fuel_Pump_V3_1_20201113_20KHz_sfun.mak
MATLAB_ROOT  = E:\Application\MATLAB2019B\matlab2019b
BUILDARGS   =

#--------------------------- Tool Specifications ------------------------------
#
#
MSVC_ROOT1 = $(MSDEVDIR:SharedIDE=vc)
MSVC_ROOT2 = $(MSVC_ROOT1:SHAREDIDE=vc)
MSVC_ROOT  = $(MSVC_ROOT2:sharedide=vc)

# Compiler tool locations, CC, LD, LIBCMD:
CC     = cl.exe
LD     = link.exe
LIBCMD = lib.exe
#------------------------------ Include/Lib Path ------------------------------

USER_INCLUDES   =  /I "E:\HUST\code\matlab_code\embedder_coder\code_generation\fuel_pump_v3_0_20201113\code_20khz\slprj\_sfprj\fuel_pump_v3_1_20201113_20khz\_self\sfun\src" /I "E:\HUST\code\matlab_code\embedder_coder\code_generation\fuel_pump_v3_0_20201113\code_20khz"
AUX_INCLUDES   = 
MLSLSF_INCLUDES = \
    /I "E:\Application\MATLAB2019B\matlab2019b\extern\include" \
    /I "E:\Application\MATLAB2019B\matlab2019b\simulink\include" \
    /I "E:\Application\MATLAB2019B\matlab2019b\simulink\include\sf_runtime" \
    /I "E:\Application\MATLAB2019B\matlab2019b\stateflow\c\mex\include" \
    /I "E:\Application\MATLAB2019B\matlab2019b\rtw\c\src" \
    /I "E:\HUST\code\matlab_code\embedder_coder\Code_generation\Fuel_Pump_V3_0_20201113\Code_20KHz\slprj\_sfprj\Fuel_Pump_V3_1_20201113_20KHz\_self\sfun\src" 

COMPILER_INCLUDES = /I "$(MSVC_ROOT)\include"

THIRD_PARTY_INCLUDES   = 
INCLUDE_PATH = $(USER_INCLUDES) $(AUX_INCLUDES) $(MLSLSF_INCLUDES)\
 $(THIRD_PARTY_INCLUDES)
LIB_PATH     = "$(MSVC_ROOT)\lib"

CFLAGS = /c /Zp8 /GR /W3 /EHs /D_CRT_SECURE_NO_DEPRECATE /D_SCL_SECURE_NO_DEPRECATE /D_SECURE_SCL=0 /DMX_COMPAT_64 /DMATLAB_MEXCMD_RELEASE=R2018a /DMATLAB_MEX_FILE /nologo /MD  
LDFLAGS = /nologo /dll /MANIFEST /OPT:NOREF /export:mexFunction /export:mexfilerequiredapiversion  
#----------------------------- Source Files -----------------------------------

REQ_SRCS  =  $(MODEL_SRC) $(MODEL_REG) $(MODULE_SRCS)

USER_OBJS =

AUX_ABS_OBJS =

THIRD_PARTY_OBJS     = \
     "c_mexapi_version.obj" \

REQ_OBJS = $(REQ_SRCS:.cpp=.obj)
REQ_OBJS2 = $(REQ_OBJS:.c=.obj)
OBJS = $(REQ_OBJS2) $(USER_OBJS) $(AUX_ABS_OBJS) $(THIRD_PARTY_OBJS)
OBJLIST_FILE = Fuel_Pump_V3_1_20201113_20KHz_sfun.mol
SFCLIB = 
AUX_LNK_OBJS =     
USER_LIBS = 
#--------------------------------- Rules --------------------------------------

MEX_FILE_NAME_WO_EXT = $(MODEL)_$(TARGET)
MEX_FILE_NAME = $(MEX_FILE_NAME_WO_EXT).mexw64
MEX_FILE_CSF =
all : $(MEX_FILE_NAME) $(MEX_FILE_CSF)

$(MEX_FILE_NAME) : $(MAKEFILE) $(OBJS) $(SFCLIB) $(AUX_LNK_OBJS) $(USER_LIBS) $(THIRD_PARTY_LIBS)
 @echo ### Linking ...
 $(LD) $(LDFLAGS) /OUT:$(MEX_FILE_NAME) /map:"$(MEX_FILE_NAME_WO_EXT).map"\
  $(USER_LIBS) $(SFCLIB) $(AUX_LNK_OBJS)\
  $(DSP_LIBS) $(THIRD_PARTY_LIBS)\
  @$(OBJLIST_FILE)
     mt -outputresource:"$(MEX_FILE_NAME);2" -manifest "$(MEX_FILE_NAME).manifest"
	@echo ### Created $@

.c.obj :
	@echo ### Compiling "$<"
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "$<"

.cpp.obj :
	@echo ### Compiling "$<"
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "$<"


c_mexapi_version.obj :  "E:\Application\MATLAB2019B\matlab2019b\extern\version\c_mexapi_version.c"
	@echo ### Compiling "E:\Application\MATLAB2019B\matlab2019b\extern\version\c_mexapi_version.c"
	$(CC) $(CFLAGS) $(INCLUDE_PATH) "E:\Application\MATLAB2019B\matlab2019b\extern\version\c_mexapi_version.c"
