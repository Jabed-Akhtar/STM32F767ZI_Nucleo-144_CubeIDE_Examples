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


BUILD_ARTIFACT_NAME := Timer_TIM10_Basic_LEDBlink
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Timer_TIM10_Basic_LEDBlink.elf \

MAP_FILES += \
Timer_TIM10_Basic_LEDBlink.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Timer_TIM10_Basic_LEDBlink.list \


# All Target
all: main-build

# Main-build Target
main-build: Timer_TIM10_Basic_LEDBlink.elf secondary-outputs

# Tool invocations
Timer_TIM10_Basic_LEDBlink.elf Timer_TIM10_Basic_LEDBlink.map: $(OBJS) $(USER_OBJS) D:\GitHubDIR\STM32F767ZI_Nucleo-144_CubeIDE_Examples_ws\Timer_TIM10_Basic_LEDBlink\STM32F767ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Timer_TIM10_Basic_LEDBlink.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"D:\GitHubDIR\STM32F767ZI_Nucleo-144_CubeIDE_Examples_ws\Timer_TIM10_Basic_LEDBlink\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Timer_TIM10_Basic_LEDBlink.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Timer_TIM10_Basic_LEDBlink.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Timer_TIM10_Basic_LEDBlink.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Timer_TIM10_Basic_LEDBlink.elf Timer_TIM10_Basic_LEDBlink.list Timer_TIM10_Basic_LEDBlink.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
