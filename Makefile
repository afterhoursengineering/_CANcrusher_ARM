# The name of your project (used to name the compiled .hex file)
TARGET = main

# configurable options
OPTIONS = -D__arm__ -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603

# Object files to be cleaned on 'make clean'
RMOBJECTS := *.o *.d $(TARGET).elf $(TARGET).hex

# Execute the python script to configure the build directory
#PYTHONSCRIPT :=  python ../config.py
#BLANK := $(shell $(PYTHONSCRIPT))

RMCMD = rm
MYDIR = echo %cd%
TOOLSPATH = ../tools_win
# path location for the arm-none-eabi compiler
COMPILERPATH = $(TOOLSPATH)/arm-none-eabi/bin

BUILDPATH = ./build
SDFAT_PATH = ./build/SdFat
SDFAT_PATH2 = ./build/SdFat/utility

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -fdata-sections -ffunction-sections -mcpu=cortex-m4 -mthumb -nostdlib -MMD $(OPTIONS) -I. -I$(BUILDPATH) -I$(SDFAT_PATH) -I$(SDFAT_PATH2)/

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

# linker options
LDFLAGS = -Os -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1431352418 -mcpu=cortex-m4 -mthumb -T$(BUILDPATH)/mk20dx256.ld

# additional libraries to link, math
LIBS = -lm

# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# All the source file names BUILDPATH are gathered
C_FILES := $(wildcard *.c) $(wildcard $(addprefix $(BUILDPATH)/, *.c)) $(wildcard $(addprefix $(SDFAT_PATH)/, *.c)) $(wildcard $(addprefix $(SDFAT_PATH2)/, *.c))

CPP_FILES := $(wildcard *.cpp) $(wildcard $(addprefix $(BUILDPATH)/, *.cpp)) $(wildcard $(addprefix $(SDFAT_PATH)/, *.cpp)) $(wildcard $(addprefix $(SDFAT_PATH2)/, *.cpp))		  

OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

# The actual makefile rules (all .o files built by GNU make's default implicit rules)

all: $(TARGET).hex

$(TARGET).elf: $(OBJS) $(BUILDPATH)/mk20dx256.ld
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS) $(L_INC)

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex -R .eeprom $< $@
	$(abspath $(TOOLSPATH))/teensy_post_compile -file=$(basename $@) -path=$(shell $(MYDIR)) -tools=$(abspath $(TOOLSPATH))
	-$(abspath $(TOOLSPATH))/teensy_reboot

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	-$(RMCMD) $(RMOBJECTS)
	-$(RMCMD) $(BUILDPATH)/*.o $(BUILDPATH)/*.d
	-$(RMCMD) $(SDFAT_PATH)/*.o $(SDFAT_PATH)/*.d
	-$(RMCMD) $(SDFAT_PATH2)/*.o $(SDFAT_PATH2)/*.d