################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := Humid_Temp_Measurement_new
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Humid_Temp_Measurement_new.elf \

MAP_FILES += \
Humid_Temp_Measurement_new.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Humid_Temp_Measurement_new.list \

OBJCOPY_HEX += \
Humid_Temp_Measurement_new.hex \

OBJCOPY_BIN += \
Humid_Temp_Measurement_new.bin \


# All Target
all: main-build

# Main-build Target
main-build: Humid_Temp_Measurement_new.elf secondary-outputs

# Tool invocations
Humid_Temp_Measurement_new.elf Humid_Temp_Measurement_new.map: $(OBJS) $(USER_OBJS) C:\GIT_Personal\STM32-Humid-Temp-Measure\Source_Code\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Humid_Temp_Measurement_new.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\GIT_Personal\STM32-Humid-Temp-Measure\Source_Code\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Humid_Temp_Measurement_new.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Humid_Temp_Measurement_new.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Humid_Temp_Measurement_new.list"
	@echo 'Finished building: $@'
	@echo ' '

Humid_Temp_Measurement_new.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "Humid_Temp_Measurement_new.hex"
	@echo 'Finished building: $@'
	@echo ' '

Humid_Temp_Measurement_new.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Humid_Temp_Measurement_new.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Humid_Temp_Measurement_new.bin Humid_Temp_Measurement_new.elf Humid_Temp_Measurement_new.hex Humid_Temp_Measurement_new.list Humid_Temp_Measurement_new.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
