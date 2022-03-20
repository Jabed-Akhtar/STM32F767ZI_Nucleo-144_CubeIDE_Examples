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


BUILD_ARTIFACT_NAME := pwm_TIM4_n_PWM_LED
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
pwm_TIM4_n_PWM_LED.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
pwm_TIM4_n_PWM_LED.list \

OBJCOPY_BIN += \
pwm_TIM4_n_PWM_LED.bin \


# All Target
all: main-build

# Main-build Target
main-build: pwm_TIM4_n_PWM_LED.elf secondary-outputs

# Tool invocations
pwm_TIM4_n_PWM_LED.elf: $(OBJS) $(USER_OBJS) P:\zz_Kos_Bussi\KosCom_N_KosProj\zzzzz_ProductsIncomeMID\p4_MCUsNMPUsProjects\projects_ws\stm32f7_cubeIDE_ws\pwm_TIM4_n_PWM_LED\STM32F767ZITX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "pwm_TIM4_n_PWM_LED.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"P:\zz_Kos_Bussi\KosCom_N_KosProj\zzzzz_ProductsIncomeMID\p4_MCUsNMPUsProjects\projects_ws\stm32f7_cubeIDE_ws\pwm_TIM4_n_PWM_LED\STM32F767ZITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="pwm_TIM4_n_PWM_LED.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

pwm_TIM4_n_PWM_LED.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "pwm_TIM4_n_PWM_LED.list"
	@echo 'Finished building: $@'
	@echo ' '

pwm_TIM4_n_PWM_LED.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "pwm_TIM4_n_PWM_LED.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout pwm_TIM4_n_PWM_LED.bin pwm_TIM4_n_PWM_LED.elf pwm_TIM4_n_PWM_LED.list
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
