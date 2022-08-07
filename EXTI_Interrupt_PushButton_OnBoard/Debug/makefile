################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := EXTI_Interrupt_PushButton_OnBoard
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
EXTI_Interrupt_PushButton_OnBoard.elf \

MAP_FILES += \
EXTI_Interrupt_PushButton_OnBoard.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
EXTI_Interrupt_PushButton_OnBoard.list \


# All Target
all: main-build

# Main-build Target
main-build: EXTI_Interrupt_PushButton_OnBoard.elf secondary-outputs

# Tool invocations
EXTI_Interrupt_PushButton_OnBoard.elf EXTI_Interrupt_PushButton_OnBoard.map: $(OBJS) $(USER_OBJS) S:\zzz_StudyProjs\ProjLearn\Project2_Master\22_VMS\CubeIDE_ws\EXTI_Interrupt_PushButton_OnBoard\STM32F767ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "EXTI_Interrupt_PushButton_OnBoard.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"S:\zzz_StudyProjs\ProjLearn\Project2_Master\22_VMS\CubeIDE_ws\EXTI_Interrupt_PushButton_OnBoard\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="EXTI_Interrupt_PushButton_OnBoard.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

EXTI_Interrupt_PushButton_OnBoard.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "EXTI_Interrupt_PushButton_OnBoard.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) EXTI_Interrupt_PushButton_OnBoard.elf EXTI_Interrupt_PushButton_OnBoard.list EXTI_Interrupt_PushButton_OnBoard.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
