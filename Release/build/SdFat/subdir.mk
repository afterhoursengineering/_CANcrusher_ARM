################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../build/SdFat/MinimumSerial.cpp \
../build/SdFat/SdFatBase.cpp \
../build/SdFat/SdFatUtil.cpp \
../build/SdFat/SdSpiCard.cpp \
../build/SdFat/SdSpiSAM3X.cpp \
../build/SdFat/SdSpiSTM32F1.cpp \
../build/SdFat/SdSpiTeensy3.cpp 

OBJS += \
./build/SdFat/MinimumSerial.o \
./build/SdFat/SdFatBase.o \
./build/SdFat/SdFatUtil.o \
./build/SdFat/SdSpiCard.o \
./build/SdFat/SdSpiSAM3X.o \
./build/SdFat/SdSpiSTM32F1.o \
./build/SdFat/SdSpiTeensy3.o 

CPP_DEPS += \
./build/SdFat/MinimumSerial.d \
./build/SdFat/SdFatBase.d \
./build/SdFat/SdFatUtil.d \
./build/SdFat/SdSpiCard.d \
./build/SdFat/SdSpiSAM3X.d \
./build/SdFat/SdSpiSTM32F1.d \
./build/SdFat/SdSpiTeensy3.d 


# Each subdirectory must supply rules for building sources it contributes
build/SdFat/%.o: ../build/SdFat/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


