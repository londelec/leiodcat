################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/74lv8153.c \
../src/board.c \
../src/main.c \
../src/mcueecfg.c \
../src/modbus.c \
../src/modbussl.c \
../src/powman.c \
../src/timer.c \
../src/usart.c 

S_UPPER_SRCS += \
../src/irqproc.S 

OBJS += \
./src/74lv8153.o \
./src/board.o \
./src/irqproc.o \
./src/main.o \
./src/mcueecfg.o \
./src/modbus.o \
./src/modbussl.o \
./src/powman.o \
./src/timer.o \
./src/usart.o 

C_DEPS += \
./src/74lv8153.d \
./src/board.d \
./src/main.d \
./src/mcueecfg.d \
./src/modbus.d \
./src/modbussl.d \
./src/powman.d \
./src/timer.d \
./src/usart.d 

S_UPPER_DEPS += \
./src/irqproc.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I../include -D_GLOBAL_DEBUG -DEEPROM_CFG -D_DISABLE_BOARD_AUTOID -DMCUTYPE=AVR -Wall -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atxmega128a1 -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Assembler'
	avr-gcc -x assembler-with-cpp -mmcu=atxmega128a1 -DF_CPU=18432000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


