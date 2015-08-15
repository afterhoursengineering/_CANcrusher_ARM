################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../build/SdFat/utility/FatFile.cpp \
../build/SdFat/utility/FatFileLFN.cpp \
../build/SdFat/utility/FatFilePrint.cpp \
../build/SdFat/utility/FatFileSFN.cpp \
../build/SdFat/utility/FatVolume.cpp \
../build/SdFat/utility/FmtNumber.cpp \
../build/SdFat/utility/StdioStream.cpp \
../build/SdFat/utility/fstream.cpp \
../build/SdFat/utility/istream.cpp \
../build/SdFat/utility/ostream.cpp 

O_SRCS += \
../build/SdFat/utility/FatFile.o \
../build/SdFat/utility/FatFileLFN.o \
../build/SdFat/utility/FatFilePrint.o \
../build/SdFat/utility/FatFileSFN.o \
../build/SdFat/utility/FatVolume.o \
../build/SdFat/utility/FmtNumber.o \
../build/SdFat/utility/StdioStream.o \
../build/SdFat/utility/fstream.o \
../build/SdFat/utility/istream.o \
../build/SdFat/utility/ostream.o 

OBJS += \
./build/SdFat/utility/FatFile.o \
./build/SdFat/utility/FatFileLFN.o \
./build/SdFat/utility/FatFilePrint.o \
./build/SdFat/utility/FatFileSFN.o \
./build/SdFat/utility/FatVolume.o \
./build/SdFat/utility/FmtNumber.o \
./build/SdFat/utility/StdioStream.o \
./build/SdFat/utility/fstream.o \
./build/SdFat/utility/istream.o \
./build/SdFat/utility/ostream.o 

CPP_DEPS += \
./build/SdFat/utility/FatFile.d \
./build/SdFat/utility/FatFileLFN.d \
./build/SdFat/utility/FatFilePrint.d \
./build/SdFat/utility/FatFileSFN.d \
./build/SdFat/utility/FatVolume.d \
./build/SdFat/utility/FmtNumber.d \
./build/SdFat/utility/StdioStream.d \
./build/SdFat/utility/fstream.d \
./build/SdFat/utility/istream.d \
./build/SdFat/utility/ostream.d 


# Each subdirectory must supply rules for building sources it contributes
build/SdFat/utility/%.o: ../build/SdFat/utility/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


