################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FlexCAN.c \
../alarms.c \
../os.c \
../support.c 

CPP_SRCS += \
../SdLog.cpp \
../can1.cpp \
../main.cpp \
../sdfi.cpp 

OBJS += \
./FlexCAN.o \
./SdLog.o \
./alarms.o \
./can1.o \
./main.o \
./os.o \
./sdfi.o \
./support.o 

C_DEPS += \
./FlexCAN.d \
./alarms.d \
./os.d \
./support.d 

CPP_DEPS += \
./SdLog.d \
./can1.d \
./main.d \
./sdfi.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


