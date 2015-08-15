################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../build/analog.c \
../build/keylayouts.c \
../build/math_helper.c \
../build/mk20dx128.c \
../build/nonstd.c \
../build/pins_teensy.c \
../build/ser_print.c \
../build/serial1.c \
../build/serial2.c \
../build/serial3.c \
../build/touch.c \
../build/usb_desc.c \
../build/usb_dev.c \
../build/usb_joystick.c \
../build/usb_keyboard.c \
../build/usb_mem.c \
../build/usb_midi.c \
../build/usb_mouse.c \
../build/usb_rawhid.c \
../build/usb_seremu.c \
../build/usb_serial.c 

CPP_SRCS += \
../build/HardwareSerial1.cpp \
../build/HardwareSerial2.cpp \
../build/HardwareSerial3.cpp \
../build/IntervalTimer.cpp \
../build/Print.cpp \
../build/SPI.cpp \
../build/Stream.cpp \
../build/Tone.cpp \
../build/WMath.cpp \
../build/WString.cpp \
../build/Wire.cpp \
../build/avr_emulation.cpp \
../build/new.cpp \
../build/usb_flightsim.cpp \
../build/usb_inst.cpp \
../build/yield.cpp 

S_UPPER_SRCS += \
../build/memcpy-armv7m.S 

OBJS += \
./build/HardwareSerial1.o \
./build/HardwareSerial2.o \
./build/HardwareSerial3.o \
./build/IntervalTimer.o \
./build/Print.o \
./build/SPI.o \
./build/Stream.o \
./build/Tone.o \
./build/WMath.o \
./build/WString.o \
./build/Wire.o \
./build/analog.o \
./build/avr_emulation.o \
./build/keylayouts.o \
./build/math_helper.o \
./build/memcpy-armv7m.o \
./build/mk20dx128.o \
./build/new.o \
./build/nonstd.o \
./build/pins_teensy.o \
./build/ser_print.o \
./build/serial1.o \
./build/serial2.o \
./build/serial3.o \
./build/touch.o \
./build/usb_desc.o \
./build/usb_dev.o \
./build/usb_flightsim.o \
./build/usb_inst.o \
./build/usb_joystick.o \
./build/usb_keyboard.o \
./build/usb_mem.o \
./build/usb_midi.o \
./build/usb_mouse.o \
./build/usb_rawhid.o \
./build/usb_seremu.o \
./build/usb_serial.o \
./build/yield.o 

C_DEPS += \
./build/analog.d \
./build/keylayouts.d \
./build/math_helper.d \
./build/mk20dx128.d \
./build/nonstd.d \
./build/pins_teensy.d \
./build/ser_print.d \
./build/serial1.d \
./build/serial2.d \
./build/serial3.d \
./build/touch.d \
./build/usb_desc.d \
./build/usb_dev.d \
./build/usb_joystick.d \
./build/usb_keyboard.d \
./build/usb_mem.d \
./build/usb_midi.d \
./build/usb_mouse.d \
./build/usb_rawhid.d \
./build/usb_seremu.d \
./build/usb_serial.d 

CPP_DEPS += \
./build/HardwareSerial1.d \
./build/HardwareSerial2.d \
./build/HardwareSerial3.d \
./build/IntervalTimer.d \
./build/Print.d \
./build/SPI.d \
./build/Stream.d \
./build/Tone.d \
./build/WMath.d \
./build/WString.d \
./build/Wire.d \
./build/avr_emulation.d \
./build/new.d \
./build/usb_flightsim.d \
./build/usb_inst.d \
./build/yield.d 


# Each subdirectory must supply rules for building sources it contributes
build/%.o: ../build/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

build/%.o: ../build/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -D__MK20DX256__ -DF_CPU=96000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -DTEENSYDUINO=122 -DARDUINO_ARCH_AVR -DARDUINO=10603 -D__arm__ -DKINETISK -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

build/%.o: ../build/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Assembler'
	as -I"C:\_projects\_teensy\teensy3build-master\tools_win\arm-none-eabi\arm-none-eabi\include" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


