################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
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


BUILD_ARTIFACT_NAME := timer_TIM10_basic_ledBlink
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
timer_TIM10_basic_ledBlink.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
timer_TIM10_basic_ledBlink.list \

OBJCOPY_BIN += \
timer_TIM10_basic_ledBlink.bin \


# All Target
all: main-build

# Main-build Target
main-build: timer_TIM10_basic_ledBlink.elf secondary-outputs

# Tool invocations
timer_TIM10_basic_ledBlink.elf: $(OBJS) $(USER_OBJS) P:\zz_Kos_Bussi\KosCom_N_KosProj\zzzzz_ProductsIncomeMID\p4_MCUsNMPUsProjects\projects_ws\stm32f7_cubeIDE_ws\timer_TIM10_basic_ledBlink\STM32F767ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "timer_TIM10_basic_ledBlink.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"P:\zz_Kos_Bussi\KosCom_N_KosProj\zzzzz_ProductsIncomeMID\p4_MCUsNMPUsProjects\projects_ws\stm32f7_cubeIDE_ws\timer_TIM10_basic_ledBlink\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="timer_TIM10_basic_ledBlink.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

timer_TIM10_basic_ledBlink.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "timer_TIM10_basic_ledBlink.list"
	@echo 'Finished building: $@'
	@echo ' '

timer_TIM10_basic_ledBlink.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "timer_TIM10_basic_ledBlink.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout timer_TIM10_basic_ledBlink.bin timer_TIM10_basic_ledBlink.elf timer_TIM10_basic_ledBlink.list
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
