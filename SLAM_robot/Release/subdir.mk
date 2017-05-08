################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
INO_SRCS += \
../SLAM_robot.ino 

CPP_SRCS += \
../.ino.cpp 

LINK_OBJ += \
./.ino.cpp.o 

INO_DEPS += \
./SLAM_robot.ino.d 

CPP_DEPS += \
./.ino.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
.ino.cpp.o: ../.ino.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/opt/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/cores/arduino" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/variants/mega" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/libraries/Wire" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/libraries/Wire/src" -I"/home/marvin/arduino-1.8.2/libraries/Servo" -I"/home/marvin/arduino-1.8.2/libraries/Servo/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '

SLAM_robot.o: ../SLAM_robot.ino
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/opt/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -flto -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10609 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR   -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/cores/arduino" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/variants/mega" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/libraries/Wire" -I"/home/marvin/arduino-1.8.2/hardware/arduino/avr/libraries/Wire/src" -I"/home/marvin/arduino-1.8.2/libraries/Servo" -I"/home/marvin/arduino-1.8.2/libraries/Servo/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -D__IN_ECLIPSE__=1 -x c++ "$<"  -o  "$@"
	@echo 'Finished building: $<'
	@echo ' '


