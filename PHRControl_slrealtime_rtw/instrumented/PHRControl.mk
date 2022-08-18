###########################################################################
## Makefile generated for component 'PHRControl'. 
## 
## Makefile     : PHRControl.mk
## Generated on : Thu Aug 11 15:04:22 2022
## Final product: $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = PHRControl
MAKEFILE                  = PHRControl.mk
MATLAB_ROOT               = C:/PROGRA~1/MATLAB/R2022a
MATLAB_BIN                = C:/PROGRA~1/MATLAB/R2022a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
START_DIR                 = C:/Users/celwe/OneDrive/Desktop/GITHUB~1/PLANAR~1
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = ISO_C++
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ../..
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Simulink Real-Time Toolchain
# Supported Version(s):    
# ToolchainInfo Version:   2022a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# SLREALTIME_QNX_SP_ROOT
# SLREALTIME_QNX_VERSION

#-----------
# MACROS
#-----------

QCC_TARGET             = gcc_ntox86_64

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -L$(MATLAB_ROOT)/toolbox/slrealtime/target/win64/target/lib -ltraceparser -lpps -lslrealtime_kernel -lslrealtime_platform -lslrealtime_rtps -lsocket -lboost_system -lboost_log -lpci -lopenblas -lpcap

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: QNX C Compiler
CC = qcc

# Linker: QCC Linker
LD = q++

# C++ Compiler: QNX C++ Compiler
CPP = q++

# C++ Linker: QCC C++ Linker
CPP_LD = q++

# Archiver: QNX Archiver
AR = ntox86_64-gcc-ar

# Builder: GMAKE Utility
MAKE = make


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g -O0 -finstrument-functions
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g -O0 -finstrument-functions
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
RM                  = @del /F
ECHO                = @echo
MV                  = @move
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------




#---------------------------
# Model-Specific Options
#---------------------------

CFLAGS = -c -V$(QCC_TARGET) -g -O2 -fwrapv

LDFLAGS = -V$(QCC_TARGET) -g -std=gnu++14 -stdlib=libstdc++

SHAREDLIB_LDFLAGS = -V$(QCC_TARGET) -shared -Wl,--no-undefined -g

CPPFLAGS = -c -V$(QCC_TARGET) -g -std=gnu++14 -stdlib=libstdc++ -O2 -fwrapv

CPP_LDFLAGS = -V$(QCC_TARGET) -g -std=gnu++14 -stdlib=libstdc++

CPP_SHAREDLIB_LDFLAGS = -V$(QCC_TARGET) -shared -Wl,--no-undefined -g

ARFLAGS = ruvs

MAKE_FLAGS = -f $(MAKEFILE)

###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I$(START_DIR)/PHRControl_slrealtime_rtw -I$(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/dist/include -I$(MATLAB_ROOT)/toolbox/slrealtime/target/kernel/dist/include -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -IC:/PROGRA~3/SPEEDG~1/SPEEDG~1/R2022a/940~1.3/SG_BLO~1/common/libsg -I$(MATLAB_ROOT)/toolbox/shared/can/src/scanutil -IC:/PROGRA~3/SPEEDG~1/SPEEDG~1/R2022a/940~1.3 -IC:/PROGRA~3/SPEEDG~1/SPEEDG~1/R2022a/940~1.3/SG_BLO~1/can/include -I$(START_DIR)/PHRControl_slrealtime_rtw/instrumented

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -DSIMULINK_REAL_TIME
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DEXT_MODE=1 -DMAT_FILE=0 -DONESTEPFCN=1 -DTERMFCN=1 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = 
DEFINES_OPTS = -DTID01EQ=1
DEFINES_STANDARD = -DMODEL=PHRControl -DNUMST=2 -DNCSTATES=0 -DHAVESTDIO -DRT -DUSE_RTMODEL

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(MATLAB_ROOT)/rtw/c/src/rt_matrx.c $(MATLAB_ROOT)/rtw/c/src/rt_printf.c $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl.cpp $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl_cal.cpp $(START_DIR)/PHRControl_slrealtime_rtw/can_datatype_ground.cpp $(START_DIR)/PHRControl_slrealtime_rtw/rtGetInf.cpp $(START_DIR)/PHRControl_slrealtime_rtw/rtGetNaN.cpp $(START_DIR)/PHRControl_slrealtime_rtw/rt_nonfinite.cpp $(START_DIR)/PHRControl_slrealtime_rtw/slrealtime_datatype_ground.cpp $(START_DIR)/PHRControl_slrealtime_rtw/rte_PHRControl_parameters.cpp $(START_DIR)/PHRControl_slrealtime_rtw/main.cpp C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_read_s.cpp C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_setup_s.cpp C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_status_s.cpp C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_write_s.cpp host_timer_x86.c slrealtime_code_profiling_utility_functions.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = rt_matrx.o rt_printf.o PHRControl.o PHRControl_cal.o can_datatype_ground.o rtGetInf.o rtGetNaN.o rt_nonfinite.o slrealtime_datatype_ground.o rte_PHRControl_parameters.o main.o sg_IO602_IO691_read_s.o sg_IO602_IO691_setup_s.o sg_IO602_IO691_status_s.o sg_IO602_IO691_write_s.o host_timer_x86.o slrealtime_code_profiling_utility_functions.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/dist/win64/lib/libecatinterface_slrt_x64.a $(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/dist/win64/lib/libEcMaster_slrt_x64.a $(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/dist/win64/lib/libslrealtime_libsrc_pervar_slrt_x64.a C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/include/IO602_IO691_lib_qnx710_x86_64.a

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------
# Linker
#-----------

LDFLAGS_ = -lsg_qnx710_x86_64 -LC:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/common/libsg

LDFLAGS += $(LDFLAGS_)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -lsg_qnx710_x86_64 -LC:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/common/libsg

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -lsg_qnx710_x86_64 -LC:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/common/libsg

CPP_LDFLAGS += $(CPP_LDFLAGS_)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -lsg_qnx710_x86_64 -LC:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/common/libsg

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@echo "### Successfully generated all binary outputs."


prebuild : 


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	@echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) -Wl,--start-group $(LIBS) -Wl,--end-group $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/src/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/src/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_interrupts/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_interrupts/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/toolbox/slrealtime/simulink/blocks/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(START_DIR)/PHRControl_slrealtime_rtw/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(START_DIR)/PHRControl_slrealtime_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : ../%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : ../%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/toolbox/coder/profile/src/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(MATLAB_ROOT)/toolbox/coder/profile/src/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.o : $(START_DIR)/PHRControl_slrealtime_rtw/instrumented/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.o : $(START_DIR)/PHRControl_slrealtime_rtw/instrumented/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


rt_matrx.o : $(MATLAB_ROOT)/rtw/c/src/rt_matrx.c
	$(CC) $(CFLAGS) -o $@ $<


rt_printf.o : $(MATLAB_ROOT)/rtw/c/src/rt_printf.c
	$(CC) $(CFLAGS) -o $@ $<


PHRControl.o : $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


PHRControl_cal.o : $(START_DIR)/PHRControl_slrealtime_rtw/PHRControl_cal.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


can_datatype_ground.o : $(START_DIR)/PHRControl_slrealtime_rtw/can_datatype_ground.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


rtGetInf.o : $(START_DIR)/PHRControl_slrealtime_rtw/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


rtGetNaN.o : $(START_DIR)/PHRControl_slrealtime_rtw/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


rt_nonfinite.o : $(START_DIR)/PHRControl_slrealtime_rtw/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


slrealtime_datatype_ground.o : $(START_DIR)/PHRControl_slrealtime_rtw/slrealtime_datatype_ground.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


rte_PHRControl_parameters.o : $(START_DIR)/PHRControl_slrealtime_rtw/rte_PHRControl_parameters.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


main.o : $(START_DIR)/PHRControl_slrealtime_rtw/main.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


sg_IO602_IO691_read_s.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_read_s.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


sg_IO602_IO691_setup_s.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_setup_s.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


sg_IO602_IO691_status_s.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_status_s.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


sg_IO602_IO691_write_s.o : C:/ProgramData/Speedgoat/speedgoatlib/R2022a/9.4.0.3/sg_blocks/can/sg_IO602_IO691_write_s.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(ECHO) "### Deleted all derived files."

