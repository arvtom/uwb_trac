################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
UWB\ pcb.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
UWB\ pcb.list \

OBJCOPY_BIN += \
UWB\ pcb.bin \


# All Target
all: UWB\ pcb.elf secondary-outputs

# Tool invocations
UWB\ pcb.elf: $(OBJS) $(USER_OBJS) D:\KTUnas\Arvydo\Bakalauras\cube\ pcb\UWB\ pcb\STM32F732VETX_FLASH.ld
	arm-none-eabi-gcc -o "UWB pcb.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"D:\KTUnas\Arvydo\Bakalauras\cube pcb\UWB pcb\STM32F732VETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="UWB pcb.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

UWB\ pcb.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "UWB pcb.list"
	@echo 'Finished building: $@'
	@echo ' '

UWB\ pcb.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "UWB pcb.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
