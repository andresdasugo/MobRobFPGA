#START GENERATED

# The following TYPE comment allows tools to identify the 'type' of target this 
# makefile is associated with. 
# TYPE: APP_MAKEFILE

# This following VERSION comment indicates the version of the tool used to 
# generate this makefile. A makefile variable is provided for VERSION as well. 
# ACDS_VERSION: 13.1
ACDS_VERSION := 13.1

# This following BUILD_NUMBER comment indicates the build number of the tool 
# used to generate this makefile. 
# BUILD_NUMBER: 182

# Define path to the application ELF. 
# It may be used by the makefile fragments so is defined before including them. 
# 
ELF := Rover5_FreeRTOS_debug.elf

# Paths to C, C++, and assembly source files.
C_SRCS += FreeRTOS/Common_Demo_Tasks/BlockQ.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/GenQTest.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/PollQ.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/QPeek.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/blocktim.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/comtest.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/countsem.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/death.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/dynamic.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/flash.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/integer.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/recmutex.c
C_SRCS += FreeRTOS/Common_Demo_Tasks/semtest.c
C_SRCS += FreeRTOS/ParTest/ParTest.c
C_SRCS += FreeRTOS/Source/list.c
C_SRCS += FreeRTOS/Source/portable/GCC/NiosII/port.c
C_SRCS += FreeRTOS/Source/portable/MemMang/heap_2.c
C_SRCS += FreeRTOS/Source/queue.c
C_SRCS += FreeRTOS/Source/tasks.c
C_SRCS += FreeRTOS/serial.c
C_SRCS += Rover5_FreeRTOS.c
C_SRCS += rover_comm.c
C_SRCS += rover_sensing.c
C_SRCS += rover_control.c
C_SRCS += rover_comm_msgs.c
C_SRCS += rover_pid.c
C_SRCS += rover_pose_controller.c
CXX_SRCS :=
ASM_SRCS := FreeRTOS/Source/portable/GCC/NiosII/port_asm.S


# Path to root of object file tree.
OBJ_ROOT_DIR := obj

# Options to control objdump.
CREATE_OBJDUMP := 1
OBJDUMP_INCLUDE_SOURCE := 1
OBJDUMP_FULL_CONTENTS := 0

# Options to enable/disable optional files.
CREATE_ELF_DERIVED_FILES := 0
CREATE_LINKER_MAP := 1

# Common arguments for ALT_CFLAGSs
APP_CFLAGS_DEFINED_SYMBOLS :=
APP_CFLAGS_UNDEFINED_SYMBOLS :=
APP_CFLAGS_OPTIMIZATION := -O0
APP_CFLAGS_DEBUG_LEVEL := -g
APP_CFLAGS_WARNINGS := -Wall
APP_CFLAGS_USER_FLAGS :=

APP_ASFLAGS_USER :=
APP_LDFLAGS_USER :=

# Linker options that have default values assigned later if not
# assigned here.
LINKER_SCRIPT :=
CRT0 :=
SYS_LIB :=

# Define path to the root of the BSP.
BSP_ROOT_DIR := ../Rover5_FreeRTOS_bsp/

# List of application specific include directories, library directories and library names
APP_INCLUDE_DIRS += FreeRTOS
APP_INCLUDE_DIRS += FreeRTOS/Common_Demo_Tasks
APP_INCLUDE_DIRS += FreeRTOS/Common_Demo_Tasks/include
APP_INCLUDE_DIRS += FreeRTOS/Source
APP_INCLUDE_DIRS += FreeRTOS/Source/include
APP_INCLUDE_DIRS += FreeRTOS/Source/portable/MemMang
APP_INCLUDE_DIRS += FreeRTOS/Source/portable/GCC/NiosII
APP_INCLUDE_DIRS += FreeRTOS/ParTest
APP_LIBRARY_DIRS :=
APP_LIBRARY_NAMES :=

# Pre- and post- processor settings.
BUILD_PRE_PROCESS :=
BUILD_POST_PROCESS :=

QUARTUS_PROJECT_DIR := ../../


#END GENERATED
