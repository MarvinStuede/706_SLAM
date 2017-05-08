################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -f

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include libraries/Wire/src/utility/subdir.mk
-include libraries/Wire/src/subdir.mk
-include libraries/Servo/src/stm32f4/subdir.mk
-include libraries/Servo/src/samd/subdir.mk
-include libraries/Servo/src/sam/subdir.mk
-include libraries/Servo/src/nrf52/subdir.mk
-include libraries/Servo/src/avr/subdir.mk
-include libraries/MPU/subdir.mk
-include core/core/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
/home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/arduino.ar \

ELF += \
SLAM_robot.elf \

HEX += \
SLAM_robot.hex \


# All Target
all: SLAM_robot

# Tool invocations
SLAM_robot: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"/opt/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-size" -A "/home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/SLAM_robot.elf"
	@echo 'Finished building target: $@'
	@echo ' '

/home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"/opt/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-gcc-ar" rcs    /home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

SLAM_robot.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"/opt/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/4.9.2-atmel3.5.4-arduino2/bin/avr-gcc" -Wall -Wextra -Os -Wl,--gc-sections,--relax -mmcu=atmega2560 -o "/home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/SLAM_robot.elf"   $(LINK_OBJ) $(AR)   /home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release/arduino.ar  "-L/home/marvin/eclipse_ws/706_SLAM/SLAM_robot/Release" -lm
	@echo 'Finished building: $@'
	@echo ' '

SLAM_robot.hex: $(ELF)
	@echo 'Create eeprom image'
	${A.RECIPE.OBJCOPY.EEP.PATTERN}
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
