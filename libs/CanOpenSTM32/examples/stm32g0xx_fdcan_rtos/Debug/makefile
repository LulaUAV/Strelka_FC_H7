################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include CANopenNode_STM32/subdir.mk
-include CANopenNode/storage/subdir.mk
-include CANopenNode/extra/subdir.mk
-include CANopenNode/309/subdir.mk
-include CANopenNode/305/subdir.mk
-include CANopenNode/304/subdir.mk
-include CANopenNode/303/subdir.mk
-include CANopenNode/301/subdir.mk
-include CANopenNode/subdir.mk
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


BUILD_ARTIFACT_NAME := CANOpenNode-STM32G0C1E-EV-RTOS
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
CANOpenNode-STM32G0C1E-EV-RTOS.elf \

MAP_FILES += \
CANOpenNode-STM32G0C1E-EV-RTOS.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
CANOpenNode-STM32G0C1E-EV-RTOS.list \


# All Target
all: main-build

# Main-build Target
main-build: CANOpenNode-STM32G0C1E-EV-RTOS.elf secondary-outputs

# Tool invocations
CANOpenNode-STM32G0C1E-EV-RTOS.elf CANOpenNode-STM32G0C1E-EV-RTOS.map: $(OBJS) $(USER_OBJS) fail-specified-linker-script-missing makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "CANOpenNode-STM32G0C1E-EV-RTOS.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus --specs=nosys.specs -Wl,-Map="CANOpenNode-STM32G0C1E-EV-RTOS.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

CANOpenNode-STM32G0C1E-EV-RTOS.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "CANOpenNode-STM32G0C1E-EV-RTOS.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) CANOpenNode-STM32G0C1E-EV-RTOS.elf CANOpenNode-STM32G0C1E-EV-RTOS.list CANOpenNode-STM32G0C1E-EV-RTOS.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
