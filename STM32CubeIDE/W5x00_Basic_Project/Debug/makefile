################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include ioLibrary/Internet/httpServer/subdir.mk
-include ioLibrary/Internet/TFTP/subdir.mk
-include ioLibrary/Internet/SNTP/subdir.mk
-include ioLibrary/Internet/SNMP/subdir.mk
-include ioLibrary/Internet/MQTT/MQTTPacket/src/subdir.mk
-include ioLibrary/Internet/MQTT/subdir.mk
-include ioLibrary/Internet/FTPServer/subdir.mk
-include ioLibrary/Internet/FTPClient/subdir.mk
-include ioLibrary/Internet/DNS/subdir.mk
-include ioLibrary/Internet/DHCP/subdir.mk
-include ioLibrary/Ethernet/W5500/subdir.mk
-include ioLibrary/Ethernet/W5300/subdir.mk
-include ioLibrary/Ethernet/W5200/subdir.mk
-include ioLibrary/Ethernet/W5100S/subdir.mk
-include ioLibrary/Ethernet/W5100/subdir.mk
-include ioLibrary/Ethernet/subdir.mk
-include ioLibrary/Application/loopback/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
W5x00_Basic_Project.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
W5x00_Basic_Project.list \


# All Target
all: W5x00_Basic_Project.elf secondary-outputs

# Tool invocations
W5x00_Basic_Project.elf: $(OBJS) $(USER_OBJS) I:\1_Project\__2019\20191216_io_project\W5x00_Basic\STM32CubeIDE\W5x00_Basic_Project\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "W5x00_Basic_Project.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"I:\1_Project\__2019\20191216_io_project\W5x00_Basic\STM32CubeIDE\W5x00_Basic_Project\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="W5x00_Basic_Project.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

W5x00_Basic_Project.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "W5x00_Basic_Project.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
