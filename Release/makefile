################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
leiodcat.lss \

FLASH_IMAGE += \
leiodcat.hex \

EEPROM_IMAGE += \
leiodcat.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: pre-build main-build

# Main-build Target
main-build: leiodcat.elf secondary-outputs

# Tool invocations
leiodcat.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,leiodcat.map -mmcu=atxmega128a1 -o "leiodcat.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

leiodcat.lss: leiodcat.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S leiodcat.elf  >"leiodcat.lss"
	@echo 'Finished building: $@'
	@echo ' '

leiodcat.hex: leiodcat.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex leiodcat.elf  "leiodcat.hex"
	@echo 'Finished building: $@'
	@echo ' '

leiodcat.eep: leiodcat.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex leiodcat.elf  "leiodcat.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: leiodcat.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atxmega128a1 leiodcat.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) leiodcat.elf
	-@echo ' '

pre-build:
	-./createbuilddate.sh "const char FirmwareDate[] PROGMEM"
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY: main-build pre-build

-include ../makefile.targets
