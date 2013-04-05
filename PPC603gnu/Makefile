# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
WS_ROOT_DIR := C:/WindRiver/workspace
PRJ_ROOT_DIR := $(WS_ROOT_DIR)/Robot2013



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 

IDE_DEFINES = 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = Robot2013/$(MODE_DIR)/Robot2013.out \
	 Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage.o


#Rules

# Robot2013
ifeq ($(DEBUG_MODE),1)
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Robot2013/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Robot2013/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Robot2013/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Robot2013/$(MODE_DIR)/% : IDE_DEFINES = 
Robot2013/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Robot2013/$(MODE_DIR)/% : DEFINES = 
Robot2013/$(MODE_DIR)/% : EXPAND_DBG = 0
Robot2013/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Robot2013/$(MODE_DIR)/% : CPU = PPC603
Robot2013/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Robot2013/$(MODE_DIR)/% : TOOL = gnu
Robot2013/$(MODE_DIR)/% : TOOL_PATH = 
Robot2013/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Robot2013/$(MODE_DIR)/% : LIBPATH = 
Robot2013/$(MODE_DIR)/% : LIBS = 
Robot2013/$(MODE_DIR)/% : OBJ_DIR := Robot2013/$(MODE_DIR)

OBJECTS_Robot2013 = Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage.o

Robot2013/$(MODE_DIR)/Robot2013.out : $(OBJECTS_Robot2013)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_Robot2013) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_Robot2013) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Robot2013/$(MODE_DIR)/Robot2013_compile_file : $(FILE) ;

_clean :: Robot2013/$(MODE_DIR)/Robot2013_clean

Robot2013/$(MODE_DIR)/Robot2013_clean : 
	$(TRACE_FLAG)if [ -d "Robot2013" ]; then cd "Robot2013"; rm -rf $(MODE_DIR); fi


# Robot2013_partialImage
ifeq ($(DEBUG_MODE),1)
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
Robot2013_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
Robot2013_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip 
Robot2013_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
Robot2013_partialImage/$(MODE_DIR)/% : IDE_DEFINES = 
Robot2013_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
Robot2013_partialImage/$(MODE_DIR)/% : DEFINES = 
Robot2013_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
Robot2013_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
Robot2013_partialImage/$(MODE_DIR)/% : CPU = PPC603
Robot2013_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
Robot2013_partialImage/$(MODE_DIR)/% : TOOL = gnu
Robot2013_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
Robot2013_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
Robot2013_partialImage/$(MODE_DIR)/% : LIBPATH = 
Robot2013_partialImage/$(MODE_DIR)/% : LIBS = 
Robot2013_partialImage/$(MODE_DIR)/% : OBJ_DIR := Robot2013_partialImage/$(MODE_DIR)

Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Collector.o : $(PRJ_ROOT_DIR)/Collector.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/ImageProcessing.o : $(PRJ_ROOT_DIR)/ImageProcessing.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/MyRobot.o : $(PRJ_ROOT_DIR)/MyRobot.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Shooter.o : $(PRJ_ROOT_DIR)/Shooter.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/TargetCamera.o : $(PRJ_ROOT_DIR)/TargetCamera.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/customPIDs.o : $(PRJ_ROOT_DIR)/customPIDs.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/jankyTask.o : $(PRJ_ROOT_DIR)/jankyTask.cpp $(FORCE_FILE_BUILD)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_Robot2013_partialImage = Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Collector.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/ImageProcessing.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/MyRobot.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Shooter.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/TargetCamera.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/customPIDs.o \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/jankyTask.o

Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage.o : $(OBJECTS_Robot2013_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_Robot2013_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage_compile_file : $(FILE) ;

_clean :: Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage_clean

Robot2013_partialImage/$(MODE_DIR)/Robot2013_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "Robot2013_partialImage" ]; then cd "Robot2013_partialImage"; rm -rf $(MODE_DIR); fi

force : 

DEP_FILES := Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Collector.d Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/ImageProcessing.d Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/MyRobot.d \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/Shooter.d Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/TargetCamera.d Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/customPIDs.d \
	 Robot2013_partialImage/$(MODE_DIR)/Objects/Robot2013/jankyTask.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
